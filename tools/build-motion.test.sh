#!/usr/bin/env bash
#
# tools/build-motion.test.sh — self-check for build-motion.sh.
#
# The script under test drives bazel: a build, then cqueries that name the
# outputs. Both are stubbed here — one stub on PATH answering both subcommands —
# and the subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and every file the payload is staged from is a file
# this test made. Nothing here builds anything, reaches a network, or touches
# this checkout.
#
# What is worth pinning is the payload's layout, the one file whose contents this
# script has an opinion about, and its freshness. The layout is not this script's
# choice: the launcher resolves every executable and argument in its config
# against its working directory, and every process it starts reads its
# configuration by paths relative to the same place, so a payload with a file in
# the wrong place
# is a launcher that starts nothing or a process that exits at setup on a powered
# unit. The contents are the two pinion values: nothing a run starts carries a
# pinion flag, so the logger's configuration has to restate the compiled-in
# defaults or the logger writes an empty log while the gesture runs perfectly, and
# that is a refused build here. The launcher config's app names are pinned for the
# same kind of reason: they are what the launcher calls each process's log file and
# deploy-motion.sh retypes them, so a rename in a composition has to be a refused
# build rather than three tail commands naming files that never appear. And
# deploy-motion.sh refuses a payload older than
# the newest commit, whose only answer is a rebuild, so every successful build has
# to stamp what it stages.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.
#
# TODO(build-motion-test-flake): this suite failed once, in one of seventeen
# runs, and has never reproduced; if it fails again, keep the staged tree the
# harness now retains and diagnose from that before re-running anything.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and what it finds around it.
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/cogs" "${repo}/driver" "${repo}/host"
cp -- "${script_dir}/build-motion.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/build-motion.sh"
payload="${repo}/target/motion-arm64/release"

# The configuration the payload is staged from. The subject does not name these:
# it asks Bazel, and the stub below answers with this list, which is how a cog
# that gains a config file reaches the payload without the script being touched.
# The paths are the ones the compositions spell, because the point of the layout
# case below is that these paths and the payload's paths are the same paths.
config_files=(
	cogs/clip_library.names.json
	cogs/clip_library.textproto
	cogs/mover_params.textproto
	cogs/robot_logger.textproto
	cogs/robot_logger_rates.textproto
	cogs/session_params.textproto
	driver/motord_params.textproto
	host/host_params.textproto
)
for file in "${config_files[@]}"; do
	echo "# ${file}" >"${repo}/${file}"
done

# One of them is read rather than merely staged: the subject refuses a logger
# configuration that does not restate the pinion defaults every flagless process
# uses. Written the way the real one is, with a decoy in a comment and one field
# indented, so the subject's line-oriented read has something to get wrong.
logger_config="${repo}/cogs/robot_logger.textproto"

stage_logger_config() {
	cat >"$logger_config" <<'CONFIG'
# a decoy in a comment: pinion_namespace: "motion"
log_root_dir: "/run/brenn-app/logs/motion"
  pinion_shm_root: "/dev/shm"
pinion_namespace: ""
CONFIG
}
stage_logger_config

# What the stub's cquery answers for the configuration targets. Exported as one
# string because that is all a stub needs; the cases that add and remove a file
# reassign it.
export CONFIG_FILES="${config_files[*]}"

# Bazel's outputs, in a directory the stub answers with workspace-relative paths
# into — the shape the subject resolves through the bazel-out convenience
# symlink.
outs="${repo}/bazel-out/bin"
mkdir -p -- "$outs"

generated_files=(
	brenn_reachy.cogs.system_robot.motion_robot.logger_proc.tachyon
	brenn_reachy.cogs.system_robot.motion_robot.proc.tachyon
	system_robot.motion_robot.RobotCpu_event_logger_config.tachyon
	system_robot.motion_robot.RobotCpu_channel_allocations.csv
	system_robot.motion_robot.diagnostics_database_config.tachyon
)
for file in "${generated_files[@]}"; do
	echo "generated ${file}" >"${outs}/${file}"
done

# The knobs the cases turn: which machine each binary claims to be, and whether
# the writer config is among the system target's outputs at all. The subject
# reads bytes 18 and 19 of the ELF header, so the stub below writes a header
# plausible enough for that read to be unambiguous and nothing more.
export MOTORD_MACHINE=183
export HOST_MACHINE=183
export ASK_MACHINE=183
export EXE_MACHINE=183
export LAUNCHER_MACHINE=183
export ONNX_MACHINE=183
export DROP_LOGGER_CONFIG=""
export INFO_STATUS=""

# Where the stub's `info execution_root` points, and where its ONNX Runtime
# cquery answer is rooted: outside the repository, which is the whole difference
# between that file and the payload's other members.
export ONNX_PRESENT=1
export EXECROOT="${work}/execroot"
mkdir -p -- "${EXECROOT}/external/onnxruntime_linux_aarch64/lib"

# The model weights, which come out of the same place for the same reason: they
# are fetched repositories' files, so the subject resolves them against the
# execution root too. `MODELS_PRESENT` is what a case turns off to stand for a
# fetch that left nothing behind, and `MODELS_EXTRA` adds a file the payload has
# no place for -- a model added to the fetch and not to the table that says where
# it goes.
export MODELS_PRESENT=1
export MODELS_EXTRA=""
export MODEL_FILES="melspectrogram.onnx embedding_model.onnx hey_jarvis_v0.1.onnx silero_vad.onnx"
for name in $MODEL_FILES hey_pavlov_v0.1.onnx; do
	mkdir -p -- "${EXECROOT}/external/model_${name%.onnx}/file"
done

# What the rendered launcher config calls the control process, and whether the
# harness twin names the voice host or the audio device. The subject pins each
# config's app names because each name is a log file an operator is sent to,
# because a host in the twin would be a second owner of the ports the intent
# source binds, and because a pod in it would make a motion run want a mic array;
# these are how a case renames one and how a case merges either into the wrong
# config.
export APP_CONTROL=""
export HARNESS_HOST=""
export HARNESS_POD=""
export CALLS="${work}/calls"

# The audio device's binary: the one payload member the subject does not ask
# Bazel about. It is brenn-pod's, built in that repo's arm64 container, so what
# stands in for it here is a file this test writes at the path a sibling checkout
# would leave it at -- which is what the subject's default resolves to, because
# `repo_root` is the temporary tree.
pod_checkout="${work}/brenn-pod/firmware/target/reachy-pod/payload"
mkdir -p -- "$pod_checkout"
pod_default="${pod_checkout}/reachy-pod"

# An ELF header plausible enough for the subject's machine read and nothing more,
# the same shape the bazel stub writes for the binaries it stands in for. Written
# by the cases rather than on every build, because nothing regenerates this file
# between builds the way the stub regenerates Bazel's outputs.
stage_pod_binary() {
	local at=$1 machine=$2
	mkdir -p -- "$(dirname -- "$at")"
	printf '\177ELF\002\001\001\000\000\000\000\000\000\000\000\000\002\000' >"$at"
	# shellcheck disable=SC2059  # the format string is built from the machine number
	printf "$(printf '\\%03o\\%03o' $((machine % 256)) $((machine / 256)))" >>"$at"
	printf '\001\000\000\000' >>"$at"
	chmod 0755 -- "$at"
}
stage_pod_binary "$pod_default" 183

# The knobs for the three ways Bazel can answer badly: cquery failing outright,
# cquery succeeding with nothing to say, and a build whose named output is not
# where the convenience symlink puts it — a renamed target, a configuration that
# resolves to an empty set, a disabled symlink.
export CQUERY_STATUS=""
export CQUERY_EMPTY=""
export DROP_EXE_FILE=""

# The real `install`, resolved before the stub directory goes on PATH: the stub
# below is transparent for most cases and has to hand the work to the coreutils
# binary this machine actually has. Nothing in these suites depends on where a
# tool lives.
real_install=$(command -v install) || {
	echo "no install on PATH; the staging cases cannot run" >&2
	exit 1
}
export REAL_INSTALL="$real_install"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# One stub, both subcommands. `build` records what it was asked and writes the
# two binaries; `cquery` answers one of the subject's two questions — the built
# targets' outputs, or the configuration targets' files — by which one the set
# expression names. The built answer carries the compositions' own sources, as
# the real one does, so the subject's basename matching has something to get
# wrong.
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
printf 'bazel %s\n' "$*" >>"$CALLS"
sub=$1
shift
target=""
for arg in "$@"; do
	case "$arg" in
		//*) target=$arg ;;
	esac
done
case "$sub" in
	build)
		elf() {
			printf '\177ELF\002\001\001\000\000\000\000\000\000\000\000\000\002\000' >"$1"
			printf "$(printf '\\%03o\\%03o' $(($2 % 256)) $(($2 / 256)))" >>"$1"
			printf '\001\000\000\000' >>"$1"
			chmod 0755 -- "$1"
		}
		elf bazel-out/bin/reachy_motord "$MOTORD_MACHINE"
		elf bazel-out/bin/reachy_host "$HOST_MACHINE"
		elf bazel-out/bin/reachy_ask "$ASK_MACHINE"
		elf bazel-out/bin/simplelaunch "$LAUNCHER_MACHINE"
		# A knob rather than a fixture the case moves aside: this arm runs on
		# every build, so a file removed between builds would be written back.
		if [ -n "${ONNX_PRESENT:-}" ]; then
			elf "${EXECROOT}/external/onnxruntime_linux_aarch64/lib/libonnxruntime.so.1" \
				"$ONNX_MACHINE"
		else
			rm -f -- "${EXECROOT}/external/onnxruntime_linux_aarch64/lib/libonnxruntime.so.1"
		fi
		# The weights, in the fetched repositories a digest-pinned download
		# leaves them in. Bytes rather than an ELF header: nothing links these
		# and no architecture check is asked of them.
		for model in $MODEL_FILES ${MODELS_EXTRA:-}; do
			path="${EXECROOT}/external/model_${model%.onnx}/file/${model}"
			if [ -n "${MODELS_PRESENT:-}" ]; then
				echo "weights of ${model}" >"$path"
			else
				rm -f -- "$path"
			fi
		done
		cat >bazel-out/bin/robotcpu.textproto <<CONFIG
app {
  name: "${APP_CONTROL:-proc}"
  executable: "cogs/robot_clk_exe"
}
app {
  name: "logger_proc"
  executable: "cogs/robot_clk_exe"
}
app {
  name: "motord"
  executable: "reachy_motord"
}
app {
  name: "voice_host"
  executable: "reachy_host"
}
app {
  name: "pod"
  executable: "reachy_pod"
  args: "run"
}
pre_launch {
  name: "clockwork_prelaunch"
  executable: "clockwork/launch/clockwork_prelaunch.sh"
}
CONFIG
		cat >bazel-out/bin/robotcpu_harness.textproto <<CONFIG
app {
  name: "${APP_CONTROL:-proc}"
  executable: "cogs/robot_clk_exe"
}
app {
  name: "logger_proc"
  executable: "cogs/robot_clk_exe"
}
app {
  name: "motord"
  executable: "reachy_motord"
}
CONFIG
		if [ -n "${HARNESS_HOST:-}" ]; then
			cat >>bazel-out/bin/robotcpu_harness.textproto <<'CONFIG'
app {
  name: "voice_host"
  executable: "reachy_host"
}
CONFIG
		fi
		if [ -n "${HARNESS_POD:-}" ]; then
			cat >>bazel-out/bin/robotcpu_harness.textproto <<'CONFIG'
app {
  name: "pod"
  executable: "reachy_pod"
  args: "run"
}
CONFIG
		fi
		cat >>bazel-out/bin/robotcpu_harness.textproto <<'CONFIG'
pre_launch {
  name: "clockwork_prelaunch"
  executable: "clockwork/launch/clockwork_prelaunch.sh"
}
CONFIG
		printf '#!/bin/bash\n' >bazel-out/bin/clockwork_prelaunch.sh
		if [ -n "${DROP_EXE_FILE:-}" ]; then
			rm -f -- bazel-out/bin/robot_clk_exe
		else
			elf bazel-out/bin/robot_clk_exe "$EXE_MACHINE"
		fi
		;;
	cquery)
		[ -z "${CQUERY_STATUS:-}" ] || exit "$CQUERY_STATUS"
		[ -z "${CQUERY_EMPTY:-}" ] || exit 0
		case "$target" in
			*robot_config_files*)
				for f in $CONFIG_FILES; do
					echo "$f"
				done
				;;
			*system_robot_clk*)
				echo bazel-out/bin/reachy_motord
				echo bazel-out/bin/reachy_host
				echo bazel-out/bin/reachy_ask
				echo bazel-out/bin/robot_clk_exe
				echo bazel-out/bin/simplelaunch
				echo bazel-out/bin/robotcpu.textproto
				echo bazel-out/bin/robotcpu_harness.textproto
				echo bazel-out/bin/clockwork_prelaunch.sh
				echo cogs/robot.clk
				echo cogs/system_robot.clk
				for f in bazel-out/bin/*.tachyon bazel-out/bin/*.csv; do
					case "$f" in
						*event_logger_config*)
							[ -z "${DROP_LOGGER_CONFIG:-}" ] || continue
							;;
					esac
					echo "$f"
				done
				;;
			*onnxruntime*)
				# Execroot-relative, as the real cquery answers for a file
				# inside a fetched repository.
				echo external/onnxruntime_linux_aarch64/lib/libonnxruntime.so.1
				;;
			*third_party/models*)
				# Execroot-relative for the same reason.
				for model in $MODEL_FILES ${MODELS_EXTRA:-}; do
					echo "external/model_${model%.onnx}/file/${model}"
				done
				;;
			*) echo "unstubbed target ${target}" >&2; exit 1 ;;
		esac
		;;
	info)
		[ -z "${INFO_STATUS:-}" ] || exit "$INFO_STATUS"
		echo "$EXECROOT"
		;;
	*) echo "unstubbed subcommand ${sub}" >&2; exit 1 ;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/bazel"

# A second stub, transparent unless a case asks for it: `install` doing the real
# thing until the Nth call and dying on it. That is the one way staging can fail
# for a reason the subject cannot decide in advance — a full disk, a permission,
# a signal — and the case below is about what the payload looks like afterwards.
export INSTALL_FAIL_AFTER=""
export INSTALL_COUNT="${work}/installs"

cat >"${stubs}/install" <<'STUB'
#!/usr/bin/env bash
if [ -n "${INSTALL_FAIL_AFTER:-}" ]; then
	n=$(($(cat -- "$INSTALL_COUNT" 2>/dev/null || echo 0) + 1))
	echo "$n" >"$INSTALL_COUNT"
	if [ "$n" -gt "$INSTALL_FAIL_AFTER" ]; then
		echo "install: stub failure on call ${n}" >&2
		exit 1
	fi
fi
exec "$REAL_INSTALL" "$@"
STUB
chmod 0755 -- "${stubs}/install"

# A third stub: `git`, answering the two questions the subject asks it — which
# commit the payload is being staged from, and what the brenn-pod checkout
# beside this tree holds. The temporary tree has no history of its own, and what
# the push does with an answer of `unknown` is deploy-motion.sh's business; here
# the knobs are what let each answer be a case.
#
# Every arm is keyed on the *directory* the question was put to, and the two
# trees answer different revisions. A stub that answered one constant to any
# `-C` would pass every case below while the subject asked the wrong tree — or
# no tree — so the directory is what the cases actually hold.
export GIT_HEAD=0123456789abcdef0123456789abcdef01234567
# This tree, as the subject spells it when it asks for the build stamp.
export REPO_DIR="$repo"
export POD_GIT_HEAD=89abcdef0123456789abcdef0123456789abcdef
# The brenn-pod checkout as the subject spells it: `lib.sh` resolves the default
# relative to the repository root, and does not canonicalise it.
export POD_DIR="${repo}/../brenn-pod"
# Where that checkout's `rev-parse --show-toplevel` lands. Itself for an
# ordinary checkout; a case points it at an enclosing directory to stand for a
# tree that is not a repository root but sits inside one.
export POD_TOPLEVEL="${work}/brenn-pod"
# The date of that checkout's HEAD commit, against which the staged binary's
# mtime is read. Well in the past, so a fixture file written by this suite is
# newer than it until a case says otherwise.
export POD_COMMIT_TIME=1000000000

cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
# -C <dir> rev-parse HEAD, -C <dir> rev-parse --show-toplevel, and
# -C <dir> log -1 --format=%ct. Nothing else is asked.
case "$*" in
"-C ${POD_DIR} rev-parse HEAD")
	[ -n "${POD_GIT_HEAD:-}" ] || exit 128
	echo "$POD_GIT_HEAD"
	;;
"-C ${POD_DIR} rev-parse --show-toplevel")
	[ -n "${POD_TOPLEVEL:-}" ] || exit 128
	echo "$POD_TOPLEVEL"
	;;
"-C ${POD_DIR} log -1 --format=%ct")
	[ -n "${POD_COMMIT_TIME:-}" ] || exit 128
	echo "$POD_COMMIT_TIME"
	;;
"-C ${REPO_DIR} rev-parse HEAD")
	[ -n "${GIT_HEAD:-}" ] || exit 128
	echo "$GIT_HEAD"
	;;
# Any other directory is not a repository this stub knows: git's own answer for
# a tree with no history, which is what the cases that move a knob are asking
# the subject to handle.
*rev-parse*|*log*) exit 128 ;;
*) echo "unstubbed git ${*}" >&2; exit 1 ;;
esac
STUB
chmod 0755 -- "${stubs}/git"

build() {
	: >"$CALLS"
	: >"$INSTALL_COUNT"
	local out status=0
	out=$(cd -- "$repo" && "$subject" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

calls() { cat -- "$CALLS"; }

# ---------------------------------------------------------------------------
# The payload's layout
# ---------------------------------------------------------------------------

result=$(build)
assert_status "a clean build succeeds" 0 "$(status_of "$result")"

assert_file "the driver is in the payload" "${payload}/reachy_motord"
assert_file "the voice host is beside it" "${payload}/reachy_host"
# The one member the subject stages from outside Bazel's outputs, found at the
# path a sibling brenn-pod checkout leaves it at, with no knob set.
assert_file "the audio device is beside them" "${payload}/reachy_pod"
assert_file "the intent source is beside it" "${payload}/reachy_ask"
assert_file "the launcher is in the payload" "${payload}/simplelaunch"
# Beside the host and not under a lib directory: the binary's runpath ends in
# `$ORIGIN`, and the payload root is where that resolves.
assert_file "the shared object the host needs is beside it" \
	"${payload}/libonnxruntime.so.1"
assert_file "its config is beside it, where it is started from" \
	"${payload}/robotcpu.textproto"
# Both configs travel: a unit is deployed once and used for production presence
# and for a motion run, and `--run` names the twin.
assert_file "the harness twin travels with it" \
	"${payload}/robotcpu_harness.textproto"

# The three paths the launcher config spells, and the only reason they are these
# paths: the executable under `cogs/`, the prelaunch script under the directory
# upstream's own AppConfig names, and the launcher's own config at the root it is
# started from.
assert_file "the executable is where the config's apps name it" \
	"${payload}/cogs/robot_clk_exe"
assert_no_file "and not at the payload root any more" "${payload}/robot_clk_exe"
assert_file "the prelaunch script is where the config names it" \
	"${payload}/clockwork/launch/clockwork_prelaunch.sh"

if [ -x "${payload}/clockwork/launch/clockwork_prelaunch.sh" ]; then
	pass "and the launcher can execute it"
else
	fail "and the launcher can execute it" "it is staged without a mode bit"
fi
for file in "${config_files[@]}"; do
	assert_file "the payload carries ${file} at that path" "${payload}/${file}"
done
# The weights, at the paths a speech configuration names them by — the wake
# gate's three under one directory and the endpointer's under another, because
# that is how the configuration that loads them spells the paths.
assert_file "the wake gate's spectrogram model is staged" \
	"${payload}/models/oww/melspectrogram.onnx"
assert_file "its embedding model is beside it" \
	"${payload}/models/oww/embedding_model.onnx"
assert_file "the wake phrase's own model is beside them" \
	"${payload}/models/oww/hey_jarvis_v0.1.onnx"
assert_file "the endpointer's model is staged" \
	"${payload}/models/silero/silero_vad.onnx"
assert_no_file "and none of them is left at the payload root" \
	"${payload}/silero_vad.onnx"

assert_file "the writer's channel set is staged" \
	"${payload}/cogs/system_robot.motion_robot.RobotCpu_event_logger_config.tachyon"
assert_file "the control process description is staged" \
	"${payload}/cogs/brenn_reachy.cogs.system_robot.motion_robot.proc.tachyon"
assert_file "the logger process description is staged" \
	"${payload}/cogs/brenn_reachy.cogs.system_robot.motion_robot.logger_proc.tachyon"

# What the push cannot work out for itself: the commit these binaries came out
# of. A payload built here and pushed from another checkout would otherwise be
# stamped with the pushing tree's HEAD, which never produced it, and the
# freshness refusal there only turns away a payload that is too old.
assert_file "the payload names the commit it was built from" \
	"${payload}/build-commit.txt"
assert_eq "and it is the commit the tree was at" "commit=${GIT_HEAD}" \
	"$(cat -- "${payload}/build-commit.txt")"

# A tree with no history is not a build refusal — provenance is the push's
# refusal to make — but it is not a guess either.
GIT_HEAD=""
result=$(build)
assert_status "a build in a tree that cannot state its commit succeeds" 0 \
	"$(status_of "$result")"
assert_eq "and says so rather than naming one" "commit=unknown" \
	"$(cat -- "${payload}/build-commit.txt")"
GIT_HEAD=0123456789abcdef0123456789abcdef01234567
result=$(build)
assert_status "and the stamped build is back" 0 "$(status_of "$result")"

# The system target emits eleven files and the payload carries three. Everything
# else belongs to the launcher, the channel spy or the diagnostics database, none
# of which this payload starts, and pushing them would say they were needed.
assert_no_file "the launcher's channel allocations are not staged" \
	"${payload}/cogs/system_robot.motion_robot.RobotCpu_channel_allocations.csv"
assert_no_file "the diagnostics database config is not staged" \
	"${payload}/cogs/system_robot.motion_robot.diagnostics_database_config.tachyon"

# A composition's own source turns up in the built targets' listing, as it does
# in the real one. It is not a payload file, and a basename match that took the
# first line containing a name would have staged one.
assert_no_file "a composition source in the listing is not staged" \
	"${payload}/cogs/robot.clk"

assert_contains "the build asks for the device configuration" "$(calls)" "--config=device"
assert_lacks "the configuration is not spelled out here" "$(calls)" \
	"--platforms=//bazel/platform:reachy-device"
assert_contains "the build builds the deployables the gate names" "$(calls)" \
	"build --config=device -- //bazel/platform:motion_payload"
assert_contains "one cquery names every built output" "$(calls)" \
	"//crates/reachy-motord:reachy_motord + //crates/reachy-host:reachy_host + //crates/reachy-ask:reachy_ask + //cogs:robot_clk_exe + //cogs:system_robot_clk + @clockwork//jewels/simplelaunch:simplelaunch + //cogs:robotcpu.textproto + //cogs:robotcpu_harness.textproto + //cogs:clockwork_prelaunch_sh"
assert_contains "one cquery names the configuration" "$(calls)" \
	"//cogs:clip_library.names.json + //cogs:robot_config_files + //driver:motord_params.textproto + //host:host_params.textproto"
# Four, not one per file: the shared object and the model set each need one of
# their own, because their paths are relative to the execution root and the other
# two listings are not.
assert_eq "and there are four cqueries, not one per target" 4 \
	"$(calls | grep -c 'bazel cquery')"
assert_contains "the shared object is cqueried on its own" "$(calls)" \
	"//bazel/third_party/onnxruntime:shared_object"
assert_contains "and the model set on its own" "$(calls)" \
	"//bazel/third_party/models:models"
assert_contains "and its path is resolved against the execution root" "$(calls)" \
	"bazel info --config=device execution_root"
assert_contains "the report names the payload" "$(output_of "$result")" "$payload"

# The payload's contents come from Bazel, so a config file a composition gains is
# staged with no edit here. The drift this closes is invisible until a process
# dies at setup on a powered unit.
echo '# late arrival' >"${repo}/cogs/late_params.textproto"
CONFIG_FILES="${config_files[*]} cogs/late_params.textproto"
result=$(build)
assert_status "a build after a config file was added succeeds" 0 "$(status_of "$result")"
assert_file "a config file Bazel newly answers with is staged" \
	"${payload}/cogs/late_params.textproto"
CONFIG_FILES="${config_files[*]}"
result=$(build)
assert_no_file "and one it stops answering with is gone" \
	"${payload}/cogs/late_params.textproto"
rm -f -- "${repo}/cogs/late_params.textproto"

# ---------------------------------------------------------------------------
# Freshness: every successful build stamps, because a rebuild is the only answer
# deploy-motion.sh's refusal has.
# ---------------------------------------------------------------------------

old=1600000000
touch -d "@${old}" -- "${payload}/cogs/robot_clk_exe" "${payload}/reachy_motord" \
	"${payload}/cogs/session_params.textproto"
result=$(build)
assert_status "a second build succeeds" 0 "$(status_of "$result")"
for file in cogs/robot_clk_exe reachy_motord simplelaunch \
	cogs/session_params.textproto; do
	stamped=$(stat -c %Y -- "${payload}/${file}")
	if [ "$stamped" -gt "$old" ]; then
		pass "the build stamped ${file}"
	else
		fail "the build stamped ${file}" "still dated $(date -d "@${stamped}")"
	fi
done

# A file the payload no longer wants does not stay behind being deployed.
: >"${payload}/cogs/left_over.textproto"
result=$(build)
assert_status "a build over a stale payload succeeds" 0 "$(status_of "$result")"
assert_no_file "a file the layout dropped is gone" "${payload}/cogs/left_over.textproto"

# ---------------------------------------------------------------------------
# Refusals. A refused build must leave the previous payload alone — every
# refusal has to hold that, not only the ones decided before staging starts.
# `install` stamps what it copies, so a build that staged half a payload and then
# died leaves a fresh-looking directory that passes every downstream freshness
# check while missing files nothing verified.
# ---------------------------------------------------------------------------

marker="${payload}/cogs/session_params.textproto"

mark_payload() { touch -d "@${old}" -- "$marker"; }

assert_unstaged() {
	if [ "$(stat -c %Y -- "$marker")" = "$old" ]; then
		pass "$1"
	else
		fail "$1" "the payload was restaged anyway"
	fi
}

mark_payload
EXE_MACHINE=62 # x86-64: a platform flag that reached Rust and not C++
result=$(build)
assert_status "an executable for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the binary" "$(output_of "$result")" "robot_clk_exe is an ELF"
assert_contains "the refusal names the platform flag" "$(output_of "$result")" \
	"platform flag did not take effect"
assert_unstaged "a build refused for the wrong machine stages nothing"
EXE_MACHINE=183

mark_payload
HOST_MACHINE=62
result=$(build)
assert_status "a voice host for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the host binary" "$(output_of "$result")" \
	"reachy_host is an ELF"
assert_unstaged "and that one stages nothing either"
HOST_MACHINE=183

mark_payload
ASK_MACHINE=62
result=$(build)
assert_status "an intent source for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names that binary" "$(output_of "$result")" \
	"reachy_ask is an ELF"
assert_unstaged "and that one stages nothing either"
ASK_MACHINE=183

# The prebuilt member's two refusals. It is the one binary in the payload that
# was compiled somewhere else, so it is the one where "is this even an aarch64
# executable" is a real question: brenn-pod builds the same crate for the
# workstation as well, at a path that looks much like this one.
mark_payload
stage_pod_binary "$pod_default" 62
result=$(build)
assert_status "an audio device for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names that binary" "$(output_of "$result")" \
	"reachy-pod is an ELF"
assert_unstaged "and that one stages nothing either"
stage_pod_binary "$pod_default" 183

# Absent, which is the ordinary case in a checkout that has never built the other
# repo. The production launcher config names the pod app unconditionally, so a
# payload without the binary is a launcher starting an app that is not there.
mark_payload
mv -- "$pod_default" "${pod_default}.aside"
result=$(build)
assert_status "an audio-device binary that is not there refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path it looked at" "$(output_of "$result")" \
	"brenn-pod/firmware/target/reachy-pod/payload/reachy-pod"
# The remedy names the checkout this run actually resolved, not the sibling
# default: an operator whose checkouts are not siblings set BRENN_POD_DIR
# precisely so that a message would stop naming a directory they do not have.
assert_contains "and says whose build produces it, in the checkout it resolved" \
	"$(output_of "$result")" \
	"make -C ${repo}/../brenn-pod/firmware reachy-pod"
assert_contains "and names the knob that moves the checkout" "$(output_of "$result")" \
	"BRENN_POD_DIR"
assert_contains "and the one that names a bare artifact" "$(output_of "$result")" \
	"REACHY_POD_BINARY"
# Asked of this build, before the marker is reset for the next one:
# assert_unstaged measures against the last mark_payload, so an assertion left
# below a second build interrogates that one twice and this one not at all.
assert_unstaged "and a payload without it is never staged"

# The same refusal from a checkout somewhere else: the remedy follows the knob.
mark_payload
BRENN_POD_DIR="${work}/elsewhere-pod"
export BRENN_POD_DIR
result=$(build)
unset BRENN_POD_DIR
assert_status "a named checkout with no binary refuses too" 1 "$(status_of "$result")"
assert_contains "and the build command names that checkout" "$(output_of "$result")" \
	"make -C ${work}/elsewhere-pod/firmware reachy-pod"
assert_unstaged "and stages nothing"
mv -- "${pod_default}.aside" "$pod_default"

# The knob, for a checkout that is not a sibling: what it names is what is
# staged, and the default is not consulted.
mark_payload
elsewhere="${work}/elsewhere/reachy-pod"
stage_pod_binary "$elsewhere" 183
mv -- "$pod_default" "${pod_default}.aside"
REACHY_POD_BINARY="$elsewhere"
export REACHY_POD_BINARY
result=$(build)
assert_status "a build against the knob's artifact succeeds" 0 "$(status_of "$result")"
assert_file "and the payload carries it" "${payload}/reachy_pod"
unset REACHY_POD_BINARY
mv -- "${pod_default}.aside" "$pod_default"
result=$(build)
assert_status "and the default is back" 0 "$(status_of "$result")"

# ---------------------------------------------------------------------------
# The two brenn-pod revisions in one payload
# ---------------------------------------------------------------------------
#
# `reachy_pod` comes out of the brenn-pod checkout, and the `reachy_host` beside
# it links that repository's crates from the revision MODULE.bazel pins. Nothing
# makes the two agree, and a payload built out of two revisions fails on the
# unit as a handshake or a protocol mismatch -- a device round-trip to diagnose.
# So every build says which two revisions it used, and every shape of that line
# is a case here, including the ones where a revision cannot be read: silence
# there would read as agreement.
#
# The checkout's answer is the `git` stub's `POD_GIT_HEAD`, which is what the
# subject's `git -C <brenn-pod checkout> rev-parse HEAD` resolves to here --
# deliberately a different revision from `GIT_HEAD`, this tree's, so a line that
# named the wrong tree's answer could not pass.
module_pin="${repo}/MODULE.bazel"
stage_module_pin() { printf 'BRENN_POD_REV = "%s"\n' "$1" >"$module_pin"; }

other_rev=fedcba9876543210fedcba9876543210fedcba98

# No MODULE.bazel at all: this fixture tree has none until the next case writes
# one.
result=$(build)
assert_contains "a tree with no MODULE.bazel says the linked surface is unknown" \
	"$(output_of "$result")" "there is no MODULE.bazel"

# The file there and the pin unreadable -- a branch name, a short id, a reflowed
# assignment. A different cause from the one above and a different sentence, so
# a reader is not sent looking for a missing file.
stage_module_pin main
result=$(build)
assert_contains "a pin this cannot read is named as unreadable" \
	"$(output_of "$result")" "states no BRENN_POD_REV this can read"
assert_lacks "and is not reported as a missing file" \
	"$(output_of "$result")" "there is no MODULE.bazel"

# A pin, and a checkout that answers no revision -- an unpacked archive, or a
# tree somebody copied out of one.
stage_module_pin "$other_rev"
result=$(POD_GIT_HEAD="" build)
assert_contains "a checkout with no revision is named as unknown too" \
	"$(output_of "$result")" "answers no revision"
assert_contains "and the line still names the pin" "$(output_of "$result")" \
	"${other_rev:0:12}"

# The same, for the shape `rev-parse HEAD` answers anyway: a directory that is
# not a checkout root but sits inside some other repository, whose HEAD would be
# offered as brenn-pod's.
result=$(POD_TOPLEVEL="$work" build)
assert_contains "a directory inside another repository answers no revision" \
	"$(output_of "$result")" "answers no revision"
assert_lacks "rather than that repository's HEAD" \
	"$(output_of "$result")" "${POD_GIT_HEAD:0:12}"

# The pin and the checkout agreeing, with the staged binary newer than the
# checkout's HEAD commit. Not silent even here: HEAD is where the checkout
# stands now and the binary is what its last build left behind, so the line says
# what was compared.
stage_module_pin "$POD_GIT_HEAD"
result=$(build)
assert_status "a build whose two halves agree succeeds" 0 "$(status_of "$result")"
assert_contains "and says the checkout is at the pinned revision" \
	"$(output_of "$result")" \
	"pinned at ${POD_GIT_HEAD:0:12} and the checkout is at that revision"
assert_contains "and where the staged binary sits against that commit" \
	"$(output_of "$result")" "built after that commit"
assert_lacks "the answer is brenn-pod's tree's, not this one's" \
	"$(output_of "$result")" "${GIT_HEAD:0:12}"
# The same build's stamp names this tree, so the two questions demonstrably went
# to two directories.
assert_eq "while the build stamp names this tree" "commit=${GIT_HEAD}" \
	"$(cat -- "${payload}/build-commit.txt")"

# The pin and the checkout agreeing and the binary older than that commit: the
# case the agreement hides, because a pull or a pin bump moves HEAD and leaves
# the binary alone.
result=$(POD_COMMIT_TIME=4000000000 build)
assert_contains "a binary older than the checkout's HEAD commit is said to be" \
	"$(output_of "$result")" "older than that commit, so it was built at an earlier one"
assert_lacks "and the agreement is not left to speak for the binary" \
	"$(output_of "$result")" "built after that commit"

# A checkout whose HEAD commit has no date to read: nothing to compare, said
# rather than assumed either way.
result=$(POD_COMMIT_TIME="" build)
assert_contains "an undatable commit leaves the binary's age uncompared" \
	"$(output_of "$result")" "cannot be compared against it"

# The two disagreeing: a note and never a refusal, because a development
# checkout legitimately sits ahead of the pin.
stage_module_pin "$other_rev"
result=$(build)
assert_status "a build out of two revisions still succeeds" 0 "$(status_of "$result")"
assert_contains "and says they are two" "$(output_of "$result")" \
	"two revisions of brenn-pod in one payload"
assert_contains "naming the pinned one" "$(output_of "$result")" "${other_rev:0:12}"
assert_contains "and the checkout's" "$(output_of "$result")" "${POD_GIT_HEAD:0:12}"

# A bare artifact copied out of a build somewhere else: the checkout's revision
# says nothing about the file being staged, so the line does not offer it. The
# pin here is `other_rev`, so the checkout's revision appearing at all is the
# bug this denies.
REACHY_POD_BINARY="$elsewhere"
export REACHY_POD_BINARY
result=$(build)
assert_contains "an artifact named by the knob has no revision to compare" \
	"$(output_of "$result")" "REACHY_POD_BINARY names"
assert_lacks "so the checkout's revision is not offered as one" \
	"$(output_of "$result")" "${POD_GIT_HEAD:0:12}"
unset REACHY_POD_BINARY

stage_module_pin "$POD_GIT_HEAD"
result=$(build)
assert_status "and the agreeing pair is back" 0 "$(status_of "$result")"

# The other member from outside the build, and the only optional one: the voice
# pipeline's own configuration. It is a site's file -- speech endpoints, a bus
# token, this unit's link keys -- so it is never in the tree and a payload built
# without one is the ordinary case. The launcher entry names its path
# unconditionally, and what makes that safe is that the host survives its
# absence; what this suite can hold is that the build stages nothing there, says
# so, and refuses only when somebody asked for a file that is not there.
speech_default="${repo}/host/speech.toml"

result=$(build)
assert_status "a build with no speech configuration succeeds" 0 "$(status_of "$result")"
assert_no_file "and the payload carries none" "${payload}/host/speech.toml"
assert_contains "and the report says what that host will do" "$(output_of "$result")" \
	"edge half alone"

printf 'listen_addr = "0.0.0.0:7380"\n' >"$speech_default"
result=$(build)
assert_status "a build with one at the default path succeeds" 0 "$(status_of "$result")"
assert_file "and the payload carries it where the launcher argument names it" \
	"${payload}/host/speech.toml"
assert_eq "readable by the account that runs the payload and nobody else" 600 \
	"$(stat -c %a -- "${payload}/host/speech.toml")"
assert_contains "and the report says where it came from" "$(output_of "$result")" \
	"staged from ${speech_default}"
rm -- "$speech_default"

mark_payload
REACHY_SPEECH_CONFIG="${work}/nowhere/speech.toml"
export REACHY_SPEECH_CONFIG
result=$(build)
assert_status "a named speech configuration that is not there refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path it looked at" "$(output_of "$result")" \
	"${work}/nowhere/speech.toml"
assert_contains "and says a payload without one is a build away" "$(output_of "$result")" \
	"unset it"
assert_unstaged "and a build refused for it stages nothing"

REACHY_SPEECH_CONFIG="${work}/elsewhere/speech.toml"
mkdir -p -- "$(dirname -- "$REACHY_SPEECH_CONFIG")"
printf 'listen_addr = "0.0.0.0:7380"\n' >"$REACHY_SPEECH_CONFIG"
result=$(build)
assert_status "a build against the knob's configuration succeeds" 0 "$(status_of "$result")"
assert_file "and the payload carries it" "${payload}/host/speech.toml"
unset REACHY_SPEECH_CONFIG
result=$(build)
assert_status "and with the knob unset the default is back" 0 "$(status_of "$result")"
assert_no_file "which is not there, so neither is the payload's" \
	"${payload}/host/speech.toml"

# ---------------------------------------------------------------------------
# The credential files that configuration names
# ---------------------------------------------------------------------------
#
# A speech configuration is not one file but a small directory: the TOML, and
# the pod's key table and the bus token beside it, named by the payload-relative
# paths they will occupy. The subject stages them with it, because a payload
# member that arrived by a different route is the one file whose freshness
# nothing checks — and because the path the TOML names is the path the host
# resolves at run time, the launcher having started it at the payload root.
#
# What is pinned here is that pair of claims and the refusals that keep them
# true: the file lands where the TOML said, under the configuration's own mode,
# and every path the payload cannot honour is a refused build rather than a
# credential quietly left out.

assembly="${work}/assembly"
mkdir -p -- "${assembly}/secrets"
assembly_config="${assembly}/speech.toml"

# The whole assembly, rewritten by each case so a refusal leaves nothing behind
# for the next one. `psk` and `token` are the payload-relative paths the TOML
# names; an empty one leaves that key out of the file entirely.
stage_assembly() {
	local psk=$1 token=$2
	rm -rf -- "$assembly"
	mkdir -p -- "${assembly}/secrets"
	{
		printf 'listen_addr = "127.0.0.1:7380"\n'
		[ -z "$psk" ] || printf 'pod_psk_file = "%s"\n' "$psk"
		printf '\n[stt]\nurl = "http://speaches.example:8000"\n'
		if [ -n "$token" ]; then
			printf '\n[brenn.bridge]\ntoken_file = "%s"\n' "$token"
		fi
	} >"$assembly_config"
}

# One credential file beside the configuration, at the path the TOML names it
# by. Not written by stage_assembly, because a named-but-missing source is one
# of the refusals.
stage_credential() {
	local at="${assembly}/$1"
	mkdir -p -- "$(dirname -- "$at")"
	printf 'a credential\n' >"$at"
}

REACHY_SPEECH_CONFIG="$assembly_config"
export REACHY_SPEECH_CONFIG

stage_assembly secrets/pod-psk.toml secrets/remote.token
stage_credential secrets/pod-psk.toml
stage_credential secrets/remote.token
result=$(build)
assert_status "a build against an assembly directory succeeds" 0 "$(status_of "$result")"
assert_file "the key table lands where the configuration names it" \
	"${payload}/secrets/pod-psk.toml"
assert_file "and so does the bus token" "${payload}/secrets/remote.token"
assert_eq "the key table is readable by the payload's account and nobody else" 600 \
	"$(stat -c %a -- "${payload}/secrets/pod-psk.toml")"
assert_eq "and so is the token" 600 \
	"$(stat -c %a -- "${payload}/secrets/remote.token")"
assert_contains "the report names the key table and where it came from" \
	"$(output_of "$result")" "secrets/pod-psk.toml  staged from ${assembly}/secrets/pod-psk.toml"
assert_contains "and the token" "$(output_of "$result")" \
	"secrets/remote.token  staged from ${assembly}/secrets/remote.token"
assert_lacks "and neither line carries a digest" "$(output_of "$result")" \
	"$(sha256sum -- "${payload}/secrets/pod-psk.toml" | cut -d' ' -f1)"

# The collision check decides whether a credential would be installed over a
# payload member, and it asks that of a hand-written list. Nothing in the
# subject keeps that list in step with what `stage` actually installs, and a
# member added to one and not the other is a credential landing on top of a
# config with the winner decided by install order -- bad bytes at run time,
# found on hardware. This is the join: every path the build just staged has to
# be one the collision check knows about.
#
# The generated plan and the model weights are resolved sets rather than named
# ones, so they are recognised by the shapes and the places they occupy; the
# two credentials are this case's own. Everything else is a name the subject
# spells in that list, and a member added to `stage` and not to it makes this
# red.
listing=$(cd -- "$payload" && find . -type f | sed 's|^\./||' | sort)
fixed=$(sed -n '/^payload_fixed_members=(/,/^)/p' -- "${script_dir}/build-motion.sh" |
	sed -e '1d' -e '$d' -e 's/^[[:space:]]*//' -e 's/"//g' \
		-e "s|^\$build_commit_name\$|build-commit.txt|" \
		-e "s|^\$speech_config_path\$|host/speech.toml|")
unaccounted=""
while read -r member; do
	[ -n "$member" ] || continue
	case "$member" in
	models/*) continue ;;
	cogs/*.textproto | cogs/*.json | cogs/*.tachyon) continue ;;
	driver/*.textproto | host/*.textproto | *.tachyon) continue ;;
	secrets/pod-psk.toml | secrets/remote.token) continue ;;
	esac
	grep -qxF -- "$member" <<<"$fixed" || unaccounted="${unaccounted}${member} "
done <<<"$listing"
assert_eq "every payload member is one the credential collision check knows about" \
	"" "$unaccounted"

# A configuration with no [brenn.bridge] is a voiced, bus-less pipeline. Legal,
# and the payload carries no token: the host composes without alerts.
stage_assembly secrets/pod-psk.toml ""
stage_credential secrets/pod-psk.toml
result=$(build)
assert_status "a configuration naming no bridge builds" 0 "$(status_of "$result")"
assert_file "and the key table is still staged" "${payload}/secrets/pod-psk.toml"
assert_no_file "and no token is" "${payload}/secrets/remote.token"

# A credential the configuration names and the assembly does not hold. The
# refusal names both, because which of the two is wrong — the path or the
# missing file — is the operator's to see.
mark_payload
stage_assembly secrets/pod-psk.toml secrets/remote.token
stage_credential secrets/pod-psk.toml
result=$(build)
assert_status "a named credential that is not beside the configuration refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the key and its value" "$(output_of "$result")" \
	"token_file = secrets/remote.token"
assert_contains "and the path it looked at" "$(output_of "$result")" \
	"${assembly}/secrets/remote.token"
assert_unstaged "and it stages nothing"

# An absolute path is the workstation-era spelling: it would stat green on the
# machine the payload was built on and name nothing on the unit.
mark_payload
stage_assembly "${assembly}/secrets/pod-psk.toml" ""
stage_credential secrets/pod-psk.toml
result=$(build)
assert_status "an absolute credential path refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says the payload carries its own credentials" \
	"$(output_of "$result")" "the payload carries its own credentials"
assert_unstaged "and that one stages nothing either"

mark_payload
stage_assembly ../pod-psk.toml ""
result=$(build)
assert_status "a credential path that climbs out of the payload refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says where it would land" "$(output_of "$result")" \
	"climbs out of the payload"
assert_unstaged "and stages nothing"

# A credential named at a path the payload already carries: the install order
# decides which file wins, and the loser is a process reading the wrong bytes.
# Both kinds are refused — a member the subject installs by name, and one it
# resolved from Bazel's answer.
mark_payload
stage_assembly reachy_host ""
stage_credential reachy_host
result=$(build)
assert_status "a credential over a payload binary refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says whose path it is" "$(output_of "$result")" \
	"a payload member's own path"
assert_unstaged "and stages nothing"

mark_payload
stage_assembly host/speech.toml ""
stage_credential host/speech.toml
result=$(build)
assert_status "a credential over the speech configuration itself refuses" 1 \
	"$(status_of "$result")"
assert_contains "and names that path" "$(output_of "$result")" "host/speech.toml"
assert_unstaged "and stages nothing"

# The collision check compares names, and the filesystem resolves what a name
# comparison does not: `./robotcpu.textproto` matches no member and installs
# over the launcher config all the same, at 0600, under a green build.
mark_payload
stage_assembly ./robotcpu.textproto ""
stage_credential ./robotcpu.textproto
result=$(build)
assert_status "a credential path that resolves onto a member refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the component it will not take" \
	"$(output_of "$result")" "a . or an empty component"
assert_unstaged "and stages nothing"

mark_payload
stage_assembly models/silero/silero_vad.onnx ""
stage_credential models/silero/silero_vad.onnx
result=$(build)
assert_status "a credential over a model refuses" 1 "$(status_of "$result")"
assert_contains "and names that path too" "$(output_of "$result")" \
	"models/silero/silero_vad.onnx"
assert_unstaged "and stages nothing"

mark_payload
stage_assembly cogs/mover_params.textproto ""
stage_credential cogs/mover_params.textproto
result=$(build)
assert_status "a credential over a cog's configuration refuses" 1 "$(status_of "$result")"
assert_contains "and names it" "$(output_of "$result")" "cogs/mover_params.textproto"
assert_unstaged "and stages nothing"

# A value this reader cannot read confidently is a refused build, not a guess:
# a credential path truncated where the quoting went wrong is a plausible wrong
# file.
mark_payload
printf 'pod_psk_file = "secrets/pod-psk.toml\n' >"$assembly_config"
result=$(build)
assert_status "a credential value whose quoting does not close refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says what it could not read" "$(output_of "$result")" \
	"the value's quoting does not close"
assert_unstaged "and stages nothing"

# Back to the ordinary case, so what follows builds against a payload with no
# speech configuration at all.
unset REACHY_SPEECH_CONFIG
rm -rf -- "$assembly"
result=$(build)
assert_status "and with the assembly gone the build is voiceless again" 0 \
	"$(status_of "$result")"
assert_no_file "the payload carries no key table" "${payload}/secrets/pod-psk.toml"

mark_payload
INFO_STATUS=1
result=$(build)
assert_status "a bazel that cannot say where its execution root is refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says why that matters" "$(output_of "$result")" \
	"the fetched shared object cannot be found"
assert_unstaged "and it stages nothing"
INFO_STATUS=""

mark_payload
ONNX_PRESENT=""
result=$(build)
assert_status "a shared object the fetch did not leave behind refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path it looked at" "$(output_of "$result")" \
	"libonnxruntime.so.1 and no file is there"
assert_unstaged "and that one stages nothing either"
ONNX_PRESENT=1

mark_payload
MODELS_PRESENT=""
result=$(build)
assert_status "weights the fetch did not leave behind refuse" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path it looked at" "$(output_of "$result")" \
	"melspectrogram.onnx and no file is there"
assert_unstaged "and a payload with no weights is never staged"
MODELS_PRESENT=1

# The other direction, and the one a reader is likelier to cause: a model added
# to the fetch with nowhere to put it. Silently leaving it out would be a wake
# phrase that never fires, discovered on a unit.
mark_payload
MODELS_EXTRA=hey_pavlov_v0.1.onnx
result=$(build)
assert_status "a fetched model the payload has no place for refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names it" "$(output_of "$result")" \
	"a model called hey_pavlov_v0.1.onnx"
assert_contains "and says where to give it a place" "$(output_of "$result")" \
	"model_paths"
assert_unstaged "and that one stages nothing either"
MODELS_EXTRA=""

# The third direction: the build names fewer files than the payload has rows for
# -- a src dropped from the models filegroup, or an `http_file` removed from
# MODULE.bazel. Nothing else catches it: every file the build did name was
# resolved and placed, so the only evidence is the count.
mark_payload
MODEL_FILES="melspectrogram.onnx embedding_model.onnx hey_jarvis_v0.1.onnx"
result=$(build)
assert_status "a build naming fewer models than the payload wants refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names both counts" "$(output_of "$result")" \
	"the payload wants 4 model files and the build named 3"
assert_contains "and says which two lists have to describe one set" \
	"$(output_of "$result")" "model_paths"
assert_unstaged "and a payload short a graph is never staged"
MODEL_FILES="melspectrogram.onnx embedding_model.onnx hey_jarvis_v0.1.onnx silero_vad.onnx"

mark_payload
ONNX_MACHINE=62
result=$(build)
assert_status "a shared object for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the shared object" "$(output_of "$result")" \
	"libonnxruntime.so.1 is an ELF"
assert_unstaged "and that one stages nothing either"
ONNX_MACHINE=183

mark_payload
MOTORD_MACHINE=62
result=$(build)
assert_status "a driver for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names that binary too" "$(output_of "$result")" \
	"reachy_motord is an ELF"
assert_unstaged "and that one stages nothing either"
MOTORD_MACHINE=183

mark_payload
DROP_LOGGER_CONFIG=1
result=$(build)
assert_status "a system target with no writer config refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the file this script asked for" "$(output_of "$result")" \
	"system_robot.motion_robot.RobotCpu_event_logger_config.tachyon"
assert_unstaged "a missing generated file stages nothing"
DROP_LOGGER_CONFIG=""

# The pinion agreement. The shipped values pass — every case above built with
# them — and a drift in either field is a refused build that stages nothing,
# because nothing at run time can notice a logger looking in the wrong place.

mark_payload
sed -i 's|^pinion_namespace: .*|pinion_namespace: "motion"|' -- "$logger_config"
result=$(build)
assert_status "a logger configuration with a namespace refuses" 1 "$(status_of "$result")"
assert_contains "the refusal quotes what the file says" "$(output_of "$result")" \
	"states 'pinion_namespace: \"motion\"'"
assert_contains "and what the payload needs" "$(output_of "$result")" \
	"needs 'pinion_namespace: \"\"'"
assert_contains "and says why there is no choice about it" "$(output_of "$result")" \
	"flagless"
assert_unstaged "a drifted namespace stages nothing"
assert_lacks "and nothing was built either" "$(calls)" "bazel build"
stage_logger_config

mark_payload
sed -i 's|^  pinion_shm_root: .*|  pinion_shm_root: "/dev/shm/motion"|' -- "$logger_config"
result=$(build)
assert_status "a logger configuration with another shm root refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names that field" "$(output_of "$result")" \
	"needs 'pinion_shm_root: \"/dev/shm\"'"
assert_unstaged "a drifted shm root stages nothing"
stage_logger_config

mark_payload
sed -i '/pinion_shm_root/d' -- "$logger_config"
result=$(build)
assert_status "a logger configuration that lost the field refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says the file states nothing" "$(output_of "$result")" \
	"states 'nothing'"
assert_unstaged "a missing field stages nothing"
stage_logger_config

result=$(build)
assert_status "and the shipped values build" 0 "$(status_of "$result")"

# The launcher's app names, which are the names of the log files an operator
# tails. They come out of the compositions and `docs/bench-runbook.md` retypes
# them, so this assertion is the join between the two: a process renamed in a
# `.clk` file is a refused build here rather than a runbook naming five log
# files that never appear.

mark_payload
APP_CONTROL=control_proc
result=$(build)
assert_status "a launcher config that renamed a process refuses" 1 "$(status_of "$result")"
assert_contains "the refusal lists what the config names" "$(output_of "$result")" \
	"names the apps 'control_proc logger_proc motord pod voice_host'"
assert_contains "and what the run needs" "$(output_of "$result")" \
	"needs 'logger_proc motord pod proc voice_host'"
assert_contains "and says which config it read" "$(output_of "$result")" \
	"robotcpu.textproto names the apps"
assert_contains "and says why the names matter" "$(output_of "$result")" \
	"names each app's log file"
assert_unstaged "a renamed process stages nothing"
APP_CONTROL=""

result=$(build)
assert_status "and the rendered names build" 0 "$(status_of "$result")"

# The twin's whole reason for existing: it must not name the host. `--run`
# starts the intent source itself, and a host merged into this config would bind
# 7409 and 7410 alongside it, leaving the run's verdict to the kernel.
mark_payload
HARNESS_HOST=1
result=$(build)
assert_status "a harness twin that names the host refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the twin" "$(output_of "$result")" \
	"robotcpu_harness.textproto names the apps"
assert_contains "and lists the host among what it found" "$(output_of "$result")" \
	"'logger_proc motord proc voice_host'"
assert_contains "and what a harness run needs instead" "$(output_of "$result")" \
	"needs 'logger_proc motord proc'"
assert_unstaged "a twin naming the host stages nothing"
HARNESS_HOST=""

result=$(build)
assert_status "and the twin without it builds" 0 "$(status_of "$result")"

# The twin's second exclusion: a motion run must need no mic array, no speech
# services and nothing streaming off the board, so a pod app merged into it is
# refused for its own reason rather than tolerated as harmless.
mark_payload
HARNESS_POD=1
result=$(build)
assert_status "a harness twin that names the audio device refuses" 1 \
	"$(status_of "$result")"
assert_contains "and lists the pod among what it found" "$(output_of "$result")" \
	"'logger_proc motord pod proc'"
assert_unstaged "a twin naming the pod stages nothing"
HARNESS_POD=""

mark_payload
rm -f -- "${repo}/cogs/mover_params.textproto"
result=$(build)
assert_status "a configuration file the tree lost refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names it" "$(output_of "$result")" \
	"wants cogs/mover_params.textproto"
assert_unstaged "a configuration file the tree lost stages nothing"
echo '# back' >"${repo}/cogs/mover_params.textproto"

# ---------------------------------------------------------------------------
# Bazel answering badly. The refusals are lib.sh's and their wording is pinned
# once, in tools/lib.test.sh; what this suite owns is that each of them stops
# this script and stages nothing — a regression that turns one into a bare
# `set -e` exit leaves a payload behind.
# ---------------------------------------------------------------------------

mark_payload
CQUERY_STATUS=1
result=$(build)
assert_status "a cquery that fails refuses" 1 "$(status_of "$result")"
assert_unstaged "and stages nothing"
CQUERY_STATUS=""

mark_payload
CQUERY_EMPTY=1
result=$(build)
assert_status "a cquery that answers nothing refuses" 1 "$(status_of "$result")"
assert_unstaged "and stages nothing"
CQUERY_EMPTY=""

mark_payload
DROP_EXE_FILE=1
result=$(build)
assert_status "an output bazel names and does not put there refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path this script asked about" \
	"$(output_of "$result")" "bazel-out/bin/robot_clk_exe"
assert_unstaged "and stages nothing"
DROP_EXE_FILE=""

# ---------------------------------------------------------------------------
# A staging step that dies part-way
# ---------------------------------------------------------------------------
#
# The refusals above are the subject's own and are all decided before a byte is
# installed. This one is not decidable: `install` can die on a full disk or a
# permission at any call. What must hold anyway is that the previous payload is
# still there and still whole — an incomplete one with a fresh timestamp looks
# deployable.

mark_payload
staged_before=$(cd -- "$payload" && find . -type f | sort)
INSTALL_FAIL_AFTER=3
result=$(build)
assert_status "a build whose install dies part-way fails" 1 "$(status_of "$result")"
assert_unstaged "and the previous payload was not restaged"
assert_eq "and the previous payload is still whole" "$staged_before" \
	"$(cd -- "$payload" && find . -type f | sort)"
INSTALL_FAIL_AFTER=""

# And the half-built directory it left behind is swept rather than deployed.
result=$(build)
assert_status "the build after it succeeds" 0 "$(status_of "$result")"
assert_no_file "no half-built staging directory is left beside the payload" \
	"${payload}.new"
assert_no_file "and no payload moved aside by the swap" "${payload}.old"
assert_eq "and the payload is the layout again" "$staged_before" \
	"$(cd -- "$payload" && find . -type f | sort)"

# ---------------------------------------------------------------------------
# The shipped set and the gated set are one set
# ---------------------------------------------------------------------------
#
# Three hand-written label lists have to agree: `motion_payload`'s members, the
# labels this script cqueries the outputs of, and the list `make check-device`
# builds. The cases above cannot see a disagreement — the stub answers whatever
# it is asked, whatever the real BUILD files say — so dropping a member from
# either side leaves a deployable that no gate cross-compiles and every
# self-check green. Read out of this checkout, not the temporary tree: these are
# claims about the real files.

real_repo=$(cd -- "${script_dir}/.." && pwd)

# The labels inside one filegroup's srcs, sorted. The `name = "..."` line's own
# string is not a label and does not match, because a label starts with `//`, `:`
# or `@`; the visibility list's labels do match and are skipped by attribute
# name.
labels_of() {
	awk -v want="$1" '
		index($0, "name = \"" want "\"") { inside = 1 }
		inside && /^\)/ { inside = 0 }
		inside && /visibility/ { next }
		inside && match($0, /"(\/\/|:|@)[^"]*"/) {
			print substr($0, RSTART + 1, RLENGTH - 2)
		}
	' "${real_repo}/bazel/platform/BUILD.bazel" | sort
}

script_labels=$(grep -E '^(motord_target|host_target|ask_target|exe_target|system_target|launcher_target|launch_config_target|harness_config_target|prelaunch_target|onnx_target|models_target)=' \
	"${real_repo}/tools/build-motion.sh" | sed 's/^[a-z_]*=//' | sort)

assert_eq "the payload's members are exactly the labels this script cqueries" \
	"$(labels_of motion_payload)" "$script_labels"
assert_contains "the gate's list carries the payload" \
	"$(labels_of device_deployables)" ":motion_payload"
assert_contains "and the gate builds that list" \
	"$(cat -- "${real_repo}/Makefile")" "//bazel/platform:device_deployables"
assert_contains "and this script builds the payload filegroup itself" \
	"$(cat -- "${real_repo}/tools/build-motion.sh")" \
	"build_target=//bazel/platform:motion_payload"

# A fourth hand-written list, joined here for the same reason: the payload paths
# this script installs the weights at, and the ones deploy-motion.sh refuses a
# push without. Two scripts spelling four paths, and a path renamed in one of
# them is a push that turns away every payload this build stages.
# Each row is `<downloaded name>\t<payload path>` inside one quoted string, on an
# indented line, so the tab-separated fields are the empty indent, the name
# behind its opening quote, and the path in front of its closing one.
model_targets=$(awk -F'\t' '
	/^model_paths=\(/ { inside = 1; next }
	inside && /^\)/ { exit }
	inside && NF == 3 { print substr($3, 1, length($3) - 1) }
' "${real_repo}/tools/build-motion.sh" | sort)

deploy_models=$(awk '
	/^models=\(/ { inside = 1; next }
	inside && /^\)/ { exit }
	inside && NF { print $1 }
' "${real_repo}/tools/deploy-motion.sh" | sort)

assert_eq "the weights are staged at the four paths the push checks for" \
	"$model_targets" "$deploy_models"
assert_eq "and there are four of them, so neither list was read as empty" 4 \
	"$(printf '%s\n' "$model_targets" | grep -c .)"

# The other half of that join, and the half the cases above cannot reach: the
# fetch set itself. Everything the model section drives runs against
# `MODEL_FILES`, a list this test invented, so a `model_paths` row naming a file
# no `http_file` downloads -- or a filegroup src with no row -- is invisible to
# every case here and to `make check`, which never runs this script's subject.
# The discovery would be a refused `make motion-build` at the bench, with the
# operator already standing there. Changing the wake phrase is exactly that
# edit: one digest, one downloaded name, one row.
downloaded_names=$(awk '
	match($0, /downloaded_file_path = "[^"]*"/) {
		field = substr($0, RSTART, RLENGTH)
		match(field, /"[^"]*"/)
		print substr(field, RSTART + 1, RLENGTH - 2)
	}
' "${real_repo}/MODULE.bazel" | sort)

fetched_names=$(awk -F'\t' '
	/^model_paths=\(/ { inside = 1; next }
	inside && /^\)/ { exit }
	inside && NF == 3 { print substr($2, 2) }
' "${real_repo}/tools/build-motion.sh" | sort)

models_srcs=$(awk '
	index($0, "name = \"models\"") { inside = 1; next }
	inside && /^\)/ { exit }
	inside && match($0, /"@[^"]*"/) {
		print substr($0, RSTART + 1, RLENGTH - 2)
	}
' "${real_repo}/bazel/third_party/models/BUILD.bazel" | sort)

assert_eq "every fetched model has a row, and every row a fetch" \
	"$downloaded_names" "$fetched_names"
assert_eq "and there are four of each, so neither list was read as empty" 4 \
	"$(printf '%s\n' "$downloaded_names" | grep -c .)"
assert_eq "and the filegroup the build asks for names four repositories" 4 \
	"$(printf '%s\n' "$models_srcs" | grep -c .)"

# ---------------------------------------------------------------------------
# The two launcher config rules, held to each other
# ---------------------------------------------------------------------------
#
# `//cogs:robotcpu.textproto` is rendered by the drop's BUILD updater out of the
# system module, and `//cogs:robotcpu_harness.textproto` beside it is a
# hand-written rule repeating its inputs. So the updater edits one of the pair
# and leaves the other, and nothing else says they agree: a `data` entry lost
# from the twin costs the target its runfiles, and an `srcs` entry lost from it
# makes a motion run compose something other than what a unit ships -- discovered
# on a powered unit during `make motion-run`, which is the expensive place. The
# two rules are read out of the BUILD file here instead.

rule_attr() {
	awk -v want="$1" -v attr="$2" '
		index($0, "name = \"" want "\"") { inside = 1; next }
		inside && /^\)/ { exit }
		inside && $0 ~ "^[[:space:]]*" attr " = \\[" { collecting = 1; next }
		collecting && /^[[:space:]]*\]/ { collecting = 0; next }
		collecting && match($0, /"[^"]*"/) {
			print substr($0, RSTART + 1, RLENGTH - 2)
		}
	' "${real_repo}/cogs/BUILD.bazel" | sort
}

production_srcs=$(rule_attr robotcpu.textproto srcs)
harness_srcs=$(rule_attr robotcpu_harness.textproto srcs)

# The parse itself, so a rule renamed or an attribute restyled is a failure here
# rather than two empty lists comparing equal.
assert_contains "the production launcher rule's srcs are read out of cogs/BUILD.bazel" \
	"$production_srcs" "//driver:motord_launch.textproto"
assert_contains "and the harness twin's" \
	"$harness_srcs" "//driver:motord_launch.textproto"

production_data=$(rule_attr robotcpu.textproto data)
harness_data=$(rule_attr robotcpu_harness.textproto data)

# The same guard the srcs parse carries, for the same reason: `rule_attr` reads a
# one-entry-per-line list and nothing else, so a restyle -- a single-line list, a
# `select()`, a renamed attribute -- would hand both sides an empty string and the
# equality below would pass without having read the attribute it exists to
# compare.
assert_contains "the production launcher rule's data is read out of cogs/BUILD.bazel" \
	"$production_data" ":robot_clk_exe"
assert_contains "and the harness twin's" "$harness_data" ":robot_clk_exe"

assert_eq "the two launcher configs carry the same data" \
	"$production_data" "$harness_data"
assert_eq "the twin is the production config less the host's and the pod's app entries" \
	"$(comm -23 <(printf '%s\n' "$production_srcs") <(printf '%s\n' "$harness_srcs"))" \
	"$(printf '%s\n%s' //host:host_launch.textproto //pod:pod_launch.textproto)"
assert_eq "and names no src the production config does not" \
	"$(comm -13 <(printf '%s\n' "$production_srcs") <(printf '%s\n' "$harness_srcs"))" \
	""

# ---------------------------------------------------------------------------
# The app names and the log files the runbook tells an operator to tail
# ---------------------------------------------------------------------------
#
# The launcher writes each app's console into `<logdir>/<name>_<run>.log`, and
# `docs/bench-runbook.md` retypes those paths for a person following it by hand.
# The build refusal above cites the runbook as if it were authoritative, so the
# other half of that join is asserted here: every app the payload starts is a
# name the runbook actually tails. Read out of this checkout, both sides.

runbook=$(cat -- "${real_repo}/docs/bench-runbook.md")
shipped_apps=$(sed -n 's/^launcher_apps=(\(.*\))$/\1/p' \
	-- "${real_repo}/tools/build-motion.sh")
if [ -n "$shipped_apps" ]; then
	for app in $shipped_apps; do
		assert_contains "the runbook names ${app}'s log file" "$runbook" "${app}_0.log"
	done
else
	fail "the runbook names every app's log file" \
		"read no launcher_apps=(...) from tools/build-motion.sh — the name has moved"
fi

# ---------------------------------------------------------------------------
# The runbook against the deploy script's own vocabulary
# ---------------------------------------------------------------------------
#
# A refusal an operator meets on a unit sends them to a document, and a code in
# a status is only readable against a table. Both live in `docs/bench-runbook.md`
# and neither is generated from anything, so this is the join: every exit code
# `deploy-motion.sh` names, and every target the Makefile's help text offers, has
# to be a row that document actually carries. Read out of this checkout, all
# three sides.

deploy_src=$(cat -- "${real_repo}/tools/deploy-motion.sh")
for code in rc_no_speech_config:9 rc_no_tty:10 rc_check_refused:11 \
	rc_no_audio_conf:12 rc_service_unreachable:13; do
	name=${code%%:*}
	number=${code#*:}
	assert_contains "${name} is still ${number} in the deploy script" "$deploy_src" \
		"${name}=${number}"
	assert_contains "and the runbook tabulates ${number}" "$runbook" \
		"- **${number}** —"
done
assert_contains "the runbook names the speech run's section" "$runbook" \
	"## The speech run"
assert_contains "and the target that starts one" "$runbook" "make speech-run"
assert_contains "and the fetch that recovers its records" "$runbook" \
	"make speech-fetch"
assert_contains "and the assembly directory's payload-relative naming" "$runbook" \
	"pod_psk_file"

# The provisioning step, which no operator types and which therefore has to be
# ordered by the Makefile rather than by a runbook sentence. On a first run
# against a fresh assembly directory, provisioning is what writes the PSK table
# the build then stages, so a build that ran first would refuse a
# named-but-missing credential — and prerequisites are unordered under `-j`,
# where two recipe lines are not.
speech_recipe=$(sed -n '/^speech-run:/,/^$/p' -- "${real_repo}/Makefile")
# Make syntax, quoted so this shell leaves it alone.
# shellcheck disable=SC2016
provision_step='$(MAKE) speech-provision'
# shellcheck disable=SC2016
deploy_step='$(MAKE) motion-deploy'
assert_contains "speech-run provisions the pod's half of the link" \
	"$speech_recipe" "$provision_step"
assert_contains "and pushes the payload" "$speech_recipe" "$deploy_step"
for step in speech-provision motion-deploy; do
	assert_lacks "with ${step} not left as a prerequisite, which -j may reorder" \
		"${speech_recipe%%$'\n'*}" "$step"
done
provision_at=${speech_recipe%%"$provision_step"*}
deploy_at=${speech_recipe%%"$deploy_step"*}
if [ "${#provision_at}" -lt "${#deploy_at}" ]; then
	pass "and provisions before it builds, which is what a first run needs"
else
	fail "and provisions before it builds, which is what a first run needs" \
		"$speech_recipe"
fi

# And the cheapest refusal is the first thing asked. A stdin with no terminal
# refuses the run whatever else is true, so it is asked before the provisioning
# writes to the unit and before the build spends minutes on a payload nobody
# will run.
preflight_step="--speech-preflight"
assert_contains "speech-run asks the terminal question of its own" \
	"$speech_recipe" "$preflight_step"
preflight_at=${speech_recipe%%"$preflight_step"*}
if [ "${#preflight_at}" -lt "${#provision_at}" ]; then
	pass "and asks it before it touches the unit or builds anything"
else
	fail "and asks it before it touches the unit or builds anything" \
		"$speech_recipe"
fi

# The step `speech-run` delegates to, which is also a target of its own. Its
# recipe is one line, and every part of that line is load-bearing: the shim is
# where both refusals live, the host is what the other repository provisions,
# and `device-host` is what refuses an unnamed one before the shim hands an
# empty value across. A drop here surfaces only mid-run against hardware.
provision_recipe=$(sed -n '/^speech-provision:/,/^$/p' -- "${real_repo}/Makefile")
assert_contains "speech-provision runs this repo's shim" "$provision_recipe" \
	"tools/provision-speech.sh"
# shellcheck disable=SC2016
assert_contains "and passes it the unit" "$provision_recipe" '$(REACHY_HOST)'
assert_contains "and asks for a named unit first" "${provision_recipe%%$'\n'*}" \
	"device-host"

# The help text is where a person learns a target exists, and the runbook is
# where they learn what it does: a target offered in one and absent from the
# other is a command nobody can follow through.
makefile_help=$(sed -n '/^help:/,/^$/p' -- "${real_repo}/Makefile")
for target in speech-run speech-fetch speech-provision; do
	assert_contains "the help text offers make ${target}" "$makefile_help" \
		"make ${target}"
	assert_contains "and the runbook names make ${target}" "$runbook" \
		"make ${target}"
done

# The local configuration file, which is what lets a session type `make
# speech-run` with no variables at all. The strings below are tripwires — cheap,
# and they name the shapes a refactor most plausibly breaks — but the contract
# is make's own resolution, and that is exercised further down.
makefile_src=$(cat -- "${real_repo}/Makefile")
assert_contains "the local configuration's include is at the file's top level" \
	"$makefile_src" '
-include .local/reachy.conf'
assert_lacks "and the old guard on one variable's origin is gone" "$makefile_src" \
	'origin REACHY_HOST'

# The include's position, which the probe below cannot see: a `?=` parsed before
# it is already set when the file lands, so that variable's conf-file line is a
# silent no-op. The invariant is that every default in the Makefile comes after.
include_line=$(grep -n -e '^-include .local/reachy.conf' \
	-- "${real_repo}/Makefile" | head -n 1 | cut -d: -f1)
first_default_line=$(grep -n -E -e '^[A-Za-z_][A-Za-z0-9_]* *\?=' \
	-- "${real_repo}/Makefile" | head -n 1 | cut -d: -f1)
if [ -n "$include_line" ] && [ -n "$first_default_line" ] &&
	[ "$include_line" -lt "$first_default_line" ]; then
	pass "the include is read ahead of every default in the Makefile"
else
	fail "the include is read ahead of every default in the Makefile" \
		"-include at line [${include_line}]" \
		"first ?= at line [${first_default_line}]"
fi
assert_contains "the speech configuration is exported to the scripts" \
	"$makefile_src" 'export REACHY_SPEECH_CONFIG'
# The other repository's location: one knob for one physical fact, read by
# `tools/lib.sh` for both the pod binary it stages and the provisioning it
# invokes, and therefore exported like the speech configuration.
assert_contains "the brenn-pod checkout has a default" "$makefile_src" \
	'BRENN_POD_DIR ?='
assert_contains "and is exported to the scripts that read it" "$makefile_src" \
	'export BRENN_POD_DIR'
assert_contains "and the runbook spells the conf file's assignments with ?=" \
	"$runbook" 'REACHY_HOST ?='
assert_contains "the speech configuration among them" "$runbook" \
	'REACHY_SPEECH_CONFIG ?='
# `REACHY_HOST ?= reachy00` is make syntax and not shell syntax: sourcing the
# file leaves the variable unset and the runbook's own `ssh root@"$REACHY_HOST"`
# lines dial nothing. The document must not tell a shell to read it.
assert_lacks "and never tells the shell to source that file" "$runbook" \
	'. .local/reachy.conf'
assert_lacks "in either spelling of the same instruction" "$runbook" \
	'source .local/reachy.conf'
assert_contains "it teaches the shell's own half instead" "$runbook" \
	'export REACHY_HOST='

# The resolution itself, run rather than read. A probe makefile in this run's
# tree includes the real one, so `.local/reachy.conf` resolves against the
# probe's directory and no operator's own file is in the picture. What is
# asserted is the three-way precedence a session depends on — command line over
# environment over file over the Makefile's defaults — plus the export that
# carries a file-origin value into the scripts, which are children of a recipe
# and read it from their environment.
conf_probe="${work}/conf-probe"
mkdir -p -- "${conf_probe}/.local"
cat >"${conf_probe}/Makefile" <<'PROBE'
include @REPO@/Makefile

.PHONY: probe
probe:
	@printf 'host=[%s] bazel=[%s] speech=[%s]\n' '$(REACHY_HOST)' \
	    '$(BAZEL_FLAGS)' "$${REACHY_SPEECH_CONFIG-unset}"
PROBE
sed -i "s|@REPO@|${real_repo}|" -- "${conf_probe}/Makefile"

# The parent `make check` is a make of its own; its flags and its job server are
# no part of what is being measured here. BAZEL_FLAGS goes with them: it is a
# documented knob of this Makefile, so a shell profile or a CI `env:` block may
# export it, and an environment origin beats the conf file's `?=` — which is
# make's own rule and not what these cases are measuring.
probe_make() {
	(cd -- "$conf_probe" &&
		env -u MAKEFLAGS -u MAKELEVEL -u MFLAGS -u BAZEL_FLAGS "$@" 2>&1)
}

cat >"${conf_probe}/.local/reachy.conf" <<'CONF'
REACHY_HOST ?= from-the-file
REACHY_SPEECH_CONFIG ?= /assembly/speech.toml
BAZEL_FLAGS ?= --from-the-file
CONF

assert_eq "a conf file alone names the unit, the flags and the configuration" \
	'host=[from-the-file] bazel=[--from-the-file] speech=[/assembly/speech.toml]' \
	"$(probe_make make probe)"
assert_eq "a command-line value wins, and the file's other lines survive it" \
	'host=[argv] bazel=[--from-the-file] speech=[/assembly/speech.toml]' \
	"$(probe_make make probe REACHY_HOST=argv)"
assert_eq "an environment value wins over the file, and the rest survives" \
	'host=[from-the-env] bazel=[--from-the-file] speech=[/assembly/speech.toml]' \
	"$(probe_make REACHY_HOST=from-the-env make probe)"
assert_eq "and a command line wins over the environment" \
	'host=[argv] bazel=[--from-the-file] speech=[/assembly/speech.toml]' \
	"$(probe_make REACHY_HOST=from-the-env make probe REACHY_HOST=argv)"

# The speech configuration at its other two origins. This is the one variable
# whose whole job is reaching the scripts through the environment, and the
# operator-facing override is the documented way to point one run at another
# assembly directory; a narrowed export would drop it silently and build a
# voiceless payload from the file's config instead.
assert_eq "a command-line speech configuration reaches the scripts" \
	'host=[from-the-file] bazel=[--from-the-file] speech=[/argv/speech.toml]' \
	"$(probe_make make probe REACHY_SPEECH_CONFIG=/argv/speech.toml)"
assert_eq "and one already in the environment is passed through unchanged" \
	'host=[from-the-file] bazel=[--from-the-file] speech=[/env/speech.toml]' \
	"$(probe_make REACHY_SPEECH_CONFIG=/env/speech.toml make probe)"

rm -f -- "${conf_probe}/.local/reachy.conf"
assert_eq "with no file at all the scripts' environment is left alone" \
	'host=[] bazel=[] speech=[unset]' \
	"$(probe_make make probe)"

# The refusal an operator without a conf file meets. It teaches the `?=`
# spelling the runbook teaches — a bare `=` there shadows an environment value —
# and the runbook's half is gated above, so this is the other half.
missing_host_status=0
missing_host=$(probe_make make device-host) || missing_host_status=$?
if [ "$missing_host_status" != 0 ]; then
	pass "with no conf file, device-host refuses"
else
	fail "with no conf file, device-host refuses" "the target exited 0"
fi
assert_contains "saying which variable is missing" "$missing_host" \
	"REACHY_HOST is not set"
assert_contains "naming the file to put it in" "$missing_host" \
	".local/reachy.conf"
assert_contains "in the spelling that yields to both other origins" \
	"$missing_host" 'REACHY_HOST ?='

# The runbook's word budget. It is a checklist joined to the tools' own
# vocabulary, and every refusal in these scripts already names its own remedy,
# so prose restating that is prose that rots. A number is the only thing that
# holds against regrowth. 1000 is the budget the document is written to; room
# for the next correction comes from the document's own slack, not from a larger
# number here.
runbook_words=$(wc -w <<<"$runbook")
if [ "$runbook_words" -le 1000 ]; then
	pass "the runbook is inside its 1000-word budget"
else
	fail "the runbook is inside its 1000-word budget" \
		"wc -w says ${runbook_words}" \
		"trim the document"
fi

# ---------------------------------------------------------------------------
# The real app entries, against the names and the paths this script stages
# ---------------------------------------------------------------------------
#
# The cases above screen a launcher config this file wrote, so what they prove is
# that the screen works. The three app entries a unit actually starts are
# hand-written textprotos merged into the rendered config
# (`host/host_launch.textproto`, `driver/motord_launch.textproto`,
# `pod/pod_launch.textproto`), and their
# join to this script runs only inside `make motion-build`, which needs the
# device cross-compile. Both halves are read out of this checkout instead: an app
# renamed here has to be a name `launcher_apps` carries, and an executable
# renamed here has to be a file `stage()` installs -- otherwise the launcher
# starts an app that is not there, on a powered unit, with the runbook tailing a
# console file that never appears.

script_src=$(cat -- "${real_repo}/tools/build-motion.sh")

# Whether a whole line of a newline-separated list is exactly this string --
# `yes` or `no`, so the assertion is an equality. Substring matching would let a
# `voice_host` renamed to `host` pass against the list that still says
# `voice_host`.
member_of() {
	local needle=$1 item
	while IFS= read -r item; do
		if [ "$item" = "$needle" ]; then
			echo yes
			return
		fi
	done <<<"$2"
	echo no
}

# The paths `stage()` installs an executable at, relative to the payload root.
# Continuation lines are folded first, because one of the installs is wrapped.
staged_binaries=$(printf '%s\n' "$script_src" |
	sed -e ':a' -e '/\\$/{N;s/\\\n[[:space:]]*//;ba' -e '}' |
	sed -n 's|^[[:space:]]*install -m 0755 -D -- "[^"]*" "[^"]*}/\([^"]*\)".*$|\1|p' |
	sort -u)
assert_eq "the paths stage() installs executables at are read out of build-motion.sh" \
	yes "$(member_of reachy_motord "$staged_binaries")"

for entry in host/host_launch.textproto driver/motord_launch.textproto \
	pod/pod_launch.textproto; do
	app_name=$(sed -n 's/^ *name: "\([^"]*\)"$/\1/p' -- "${real_repo}/${entry}")
	app_exe=$(sed -n 's/^ *executable: "\([^"]*\)"$/\1/p' -- "${real_repo}/${entry}")
	if [ -z "$app_name" ] || [ -z "$app_exe" ]; then
		fail "${entry} states an app name and an executable" \
			"read name='${app_name}' executable='${app_exe}' -- the field spelling has moved"
		continue
	fi
	assert_eq "${entry}'s app name is one this script expects the config to carry" \
		yes "$(member_of "$app_name" "$(printf '%s\n' "$shipped_apps" | tr ' ' '\n')")"
	assert_eq "and its executable is a file stage() installs at the payload root" \
		yes "$(member_of "$app_exe" "$staged_binaries")"
done

# ---------------------------------------------------------------------------
# The paths the voice host resolves against the payload root
# ---------------------------------------------------------------------------
#
# `reachy_host` is a launcher app, started from the payload root, so its
# default `--config` and the `clip_names_path` inside that configuration are both
# resolved there. What puts a file at either path is `config_targets` below,
# whose members are staged at their repo-relative paths. Nothing else joins the
# two strings to that list: `crates/reachy-host/tests/shipped_params.rs` compares
# a file name and the cases above stage stubs of this test's own making, so a
# `clip_names_path` shortened to a bare file name, or a `host_params.textproto`
# moved in the tree, would pass every gate and die at setup on a powered unit.

# `//pkg:file` -> `pkg/file`, which is where the payload puts it.
staged_configs=$(sed -n '/^config_targets=(/,/^)/p' -- "${real_repo}/tools/build-motion.sh" |
	sed -n 's|^[[:space:]]*//\([^:]*\):\(.*\)$|\1/\2|p' | sort)
assert_eq "the configurations this script stages are read out of build-motion.sh" \
	yes "$(member_of driver/motord_params.textproto "$staged_configs")"

names_path=$(sed -n 's/^clip_names_path: "\([^"]*\)"$/\1/p' \
	-- "${real_repo}/host/host_params.textproto")
default_config=$(sed -n 's/^const DEFAULT_CONFIG: &str = "\([^"]*\)";$/\1/p' \
	-- "${real_repo}/crates/reachy-host/src/main.rs")
if [ -z "$names_path" ] || [ -z "$default_config" ]; then
	fail "the host's two payload-relative paths are readable" \
		"read clip_names_path='${names_path}' DEFAULT_CONFIG='${default_config}'"
else
	assert_eq "the shipped host config names a clip table the payload stages" \
		yes "$(member_of "$names_path" "$staged_configs")"
	assert_eq "and the host's default --config is a file the payload stages" \
		yes "$(member_of "$default_config" "$staged_configs")"
fi

# The third payload-relative path the host resolves, and the one no build can
# check for itself: the file `--speech-config` names is a site's own and is
# staged from outside the tree, so the only join available is the two strings --
# the argument the launcher entry spells and the path `stage()` installs it at.
# They disagree and a unit runs a host that says it is waiting for a
# configuration that is sitting right beside it.
launch_speech=$(awk -F'"' '
	/args: "--speech-config"/ { want = 1; next }
	want && /args: "/ { print $2; exit }
' "${real_repo}/host/host_launch.textproto")
staged_speech=$(sed -n 's/^speech_config_path=\(.*\)$/\1/p' \
	-- "${real_repo}/tools/lib.sh")
if [ -z "$launch_speech" ] || [ -z "$staged_speech" ]; then
	fail "the speech configuration's payload path is readable on both sides" \
		"read launcher argument='${launch_speech}' speech_config_path='${staged_speech}'"
else
	assert_eq "the launcher entry names the path the build stages one at" \
		"$staged_speech" "$launch_speech"
fi

# ---------------------------------------------------------------------------

tally
