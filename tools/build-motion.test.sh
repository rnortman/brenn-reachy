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
# against its working directory, and the three processes read their configuration
# by paths relative to the same place, so a payload with a file in the wrong place
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
mkdir -p -- "${repo}/tools" "${repo}/cogs" "${repo}/driver"
cp -- "${script_dir}/build-motion.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/build-motion.sh"
payload="${repo}/target/motion-arm64/release"

# The configuration the payload is staged from. The subject does not name these:
# it asks Bazel, and the stub below answers with this list, which is how a cog
# that gains a config file reaches the payload without the script being touched.
# The paths are the ones the compositions spell, because the point of the layout
# case below is that these paths and the payload's paths are the same paths.
config_files=(
	cogs/clip_library.textproto
	cogs/mover_params.textproto
	cogs/robot_logger.textproto
	cogs/robot_logger_rates.textproto
	cogs/session_params.textproto
	cogs/wake_params.textproto
	driver/motord_params.textproto
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
export EXE_MACHINE=183
export LAUNCHER_MACHINE=183
export DROP_LOGGER_CONFIG=""

# What the rendered launcher config calls the control process. The subject pins
# the three app names because deploy-motion.sh prints a tail command per name;
# this is how a case renames one.
export APP_CONTROL=""
export CALLS="${work}/calls"

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
		elf bazel-out/bin/simplelaunch "$LAUNCHER_MACHINE"
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
				echo bazel-out/bin/robot_clk_exe
				echo bazel-out/bin/simplelaunch
				echo bazel-out/bin/robotcpu.textproto
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
			*) echo "unstubbed target ${target}" >&2; exit 1 ;;
		esac
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

# A third stub: `git`, answering the one question the subject asks it — which
# commit the payload is being staged from. The temporary tree has no history of
# its own, and what the push does with an answer of `unknown` is
# deploy-motion.sh's business; here the knob is what lets both answers be a case.
export GIT_HEAD=0123456789abcdef0123456789abcdef01234567

cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
# -C <dir> rev-parse HEAD, and nothing else is asked.
case "$*" in
*rev-parse*HEAD*)
	[ -n "${GIT_HEAD:-}" ] || exit 128
	echo "$GIT_HEAD"
	;;
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
assert_file "the launcher is in the payload" "${payload}/simplelaunch"
assert_file "its config is beside it, where it is started from" \
	"${payload}/robotcpu.textproto"

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
	"//crates/reachy-motord:reachy_motord + //cogs:robot_clk_exe + //cogs:system_robot_clk + @clockwork//jewels/simplelaunch:simplelaunch + //cogs:robotcpu.textproto + //cogs:clockwork_prelaunch_sh"
assert_contains "one cquery names the configuration" "$(calls)" \
	"//cogs:robot_config_files + //driver:motord_params.textproto"
assert_eq "and there are two cqueries, not one per target" 2 \
	"$(calls | grep -c 'bazel cquery')"
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
# `.clk` file is a refused build here rather than three tail commands at a bench
# naming files that never appear.

mark_payload
APP_CONTROL=control_proc
result=$(build)
assert_status "a launcher config that renamed a process refuses" 1 "$(status_of "$result")"
assert_contains "the refusal lists what the config names" "$(output_of "$result")" \
	"names the apps 'control_proc logger_proc motord'"
assert_contains "and what the run needs" "$(output_of "$result")" \
	"needs 'logger_proc motord proc'"
assert_contains "and says why the names matter" "$(output_of "$result")" \
	"tail command per name"
assert_unstaged "a renamed process stages nothing"
APP_CONTROL=""

result=$(build)
assert_status "and the rendered names build" 0 "$(status_of "$result")"

mark_payload
rm -f -- "${repo}/cogs/wake_params.textproto"
result=$(build)
assert_status "a configuration file the tree lost refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names it" "$(output_of "$result")" \
	"wants cogs/wake_params.textproto"
assert_unstaged "a configuration file the tree lost stages nothing"
echo '# back' >"${repo}/cogs/wake_params.textproto"

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

script_labels=$(grep -E '^(motord_target|exe_target|system_target|launcher_target|launch_config_target|prelaunch_target)=' \
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

tally
