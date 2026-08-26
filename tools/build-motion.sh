#!/usr/bin/env bash
#
# Build the motion system for the device: the whole payload a run needs.
#
#   tools/build-motion.sh
#
# A cross-compile against the hermetic clang sysroot the pinned Clockwork drop
# brings. A motion run on a unit is three OS processes — the driver, the logger, and the
# control loop — over two binaries, and each of those processes reads
# configuration by a path relative to its working directory. So the artifact is
# not a binary but a directory laid out the way those paths expect:
#
#   target/motion-arm64/release/
#     simplelaunch                                      the launcher, started by hand
#     robotcpu.textproto                                the three apps it starts
#     clockwork/launch/clockwork_prelaunch.sh            what it runs before them
#     reachy_motord                                     the driver process
#     cogs/robot_clk_exe                                the logger and the control loop
#     driver/motord_params.textproto                    the driver's configuration
#     cogs/*.textproto                                  the cogs' configuration
#     cogs/*_event_logger_config.tachyon                which channels are written
#     cogs/*.proc.tachyon, *.logger_proc.tachyon        the two process descriptions
#
# The three launcher paths are not this script's choice: the rendered launcher
# config spells every executable and argument as a path relative to the
# launcher's working directory, which is the payload root, so where a file goes
# is what that config already says. `robot_clk_exe` lives under `cogs/` for that
# reason and no other.
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
#   REACHY_BAZEL   the bazel to run (default bazel)

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

bazel=${REACHY_BAZEL:-bazel}

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
# afterwards: the driver is a Rust binary, the executable is synthesized from
# `robot.clk` (both processes start from it), and the system target provides the
# process descriptions and the generated log-writer configuration.
motord_target=//crates/reachy-motord:reachy_motord
exe_target=//cogs:robot_clk_exe
system_target=//cogs:system_robot_clk
launcher_target=@clockwork//jewels/simplelaunch:simplelaunch
launch_config_target=//cogs:robotcpu.textproto
prelaunch_target=//cogs:clockwork_prelaunch_sh

# The configuration each process reads by a relative path, named by Bazel rather
# than by a list here. `//cogs:robot_config_files` is the same target
# `robot_clk` carries as `data`, and `//driver:motord_params.textproto` the same
# label the driver binary's `data` names -- so a cog that gains a config file is
# added in the BUILD file a developer already has to edit for the host tests to
# find it, and the payload follows. A hand-typed copy here would be a second
# source of truth whose drift is invisible until a process dies at setup on a
# powered unit.
config_targets=(
	//cogs:robot_config_files
	//driver:motord_params.textproto
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
# names, and `motord` from the app merged in through `simplelaunch_src` -- so a
# rename in a `.clk` file would leave the documented tails naming files that
# never appear.
# This is the join: the names are pinned here against the config actually built,
# and a rename is a refused build with the two lists side by side. The other
# half -- that the runbook tails a file for every name in this list -- is
# asserted by tools/build-motion.test.sh, so the refusal below cites a document
# a self-check keeps true.
launcher_apps=(logger_proc motord proc)

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
	[ "$named" = "${launcher_apps[*]}" ] || die \
		"the rendered launcher config names the apps '${named:-nothing}' and the run needs '${launcher_apps[*]}'." \
		"Those names are what the launcher calls each process's log file, and" \
		"docs/bench-runbook.md names one tail command per name. A process renamed in" \
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
stage() {
	local motord=$1 exe=$2 launcher=$3 launch_config=$4 prelaunch=$5
	local entry staging="${payload}.new" previous="${payload}.old"
	rm -rf -- "$staging" "$previous"
	mkdir -p -- "$staging"

	install -m 0755 -D -- "$motord" "${staging}/reachy_motord"
	install -m 0755 -D -- "$exe" "${staging}/cogs/robot_clk_exe"
	install -m 0755 -D -- "$launcher" "${staging}/simplelaunch"
	install -m 0644 -D -- "$launch_config" "${staging}/robotcpu.textproto"
	install -m 0755 -D -- "$prelaunch" \
		"${staging}/clockwork/launch/clockwork_prelaunch.sh"

	for entry in "${plan_files[@]}"; do
		install -m 0644 -D -- "${entry%%$'\t'*}" "${staging}/${entry#*$'\t'}"
	done

	if [ -e "$payload" ]; then
		mv -- "$payload" "$previous"
	fi
	mv -- "$staging" "$payload"
	rm -rf -- "$previous"
}

report() {
	local size
	size=$(du -sh -- "$payload" | cut -f1)
	echo "${prog}: device payload  ${payload}  (${size})"
	local file
	for file in reachy_motord cogs/robot_clk_exe simplelaunch; do
		echo "${prog}: ${file}  $(sha256sum -- "${payload}/${file}" | cut -d' ' -f1)"
	done
}

require_bazel "device payload"
check_pinion_defaults "$logger_config"
compile
built=$(bazel_files "$(union "$motord_target" "$exe_target" "$system_target" \
	"$launcher_target" "$launch_config_target" "$prelaunch_target")")
configs=$(bazel_files "$(union "${config_targets[@]}")")
motord_out=$(bazel_named_in "$built" reachy_motord)
exe_out=$(bazel_named_in "$built" robot_clk_exe)
launcher_out=$(bazel_named_in "$built" simplelaunch)
launch_config_out=$(bazel_named_in "$built" robotcpu.textproto)
prelaunch_out=$(bazel_named_in "$built" clockwork_prelaunch.sh)
check_launcher_apps "$launch_config_out"
verify_aarch64 "$motord_out"
verify_aarch64 "$exe_out"
verify_aarch64 "$launcher_out"
plan "$built" "$configs"
stage "$motord_out" "$exe_out" "$launcher_out" "$launch_config_out" "$prelaunch_out"
report
