# shellcheck shell=bash
#
# Shared prelude for the tools/ scripts. Sourced, never executed — no shebang,
# not executable:
#
#     # shellcheck source=lib.sh
#     . "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

# Everything here is read by the scripts that source this file, so "appears
# unused" is the expected shape of every definition in it.
# shellcheck disable=SC2034
#
# It also reads three variables the sourcing script owns and it cannot assign
# itself: `bazel` (which bazel to run, REACHY_BAZEL's value), `build_flags` (the
# array naming the configuration a build and its cqueries share) and `host` (the
# device being deployed to, the script's first argument). Each is named in the
# doc of every function that reads it.
# shellcheck disable=SC2154

# The name a script reports itself as in its own messages. Sourcing does not
# change $0, so this is the outer script's path.
prog=$(basename -- "$0")

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# The account the payload runs as on a brenn-os device. A bench run has to
# observe the hardware as this account or it observes nothing worth knowing:
# root opens any device node whatever udev said, so a permission assertion
# taken as root passes vacuously.
app_user=app

# The account's home on the device: writable, mode 0700, and on the volatile
# /var, so it is RAM and costs the eMMC no write. The bench's configuration and
# the state file it writes live here rather than beside the binary — the
# release directory is root-owned and rsync --delete owns its contents.
app_home=/var/lib/brenn-app

# Where brenn-os mounts the payload store. A tmpfs, so nothing under it costs
# the device's flash a write and a reboot clears it; /run itself is noexec and
# this submount deliberately is not, so a binary has to live here to be
# executable at all.
store_mount=/run/brenn-app

# The file a build script writes into the payload it stages, naming the commit
# the payload's binaries were built from. `deploy-motion.sh --push` reads it into
# the provenance stamp a run's records carry home: the pushing tree's own HEAD
# says nothing about which checkout produced the binaries, because a payload
# built at one commit can be pushed from a checkout at any other and the
# freshness refusal only catches a payload that is too old. One name here, so the
# writer and the reader cannot disagree about it.
build_commit_name=build-commit.txt

# A leading `~` replaced with $HOME, any other path unchanged. Bazel's path
# converter expands a leading `~` and so does the cache action in its path list;
# a shell passing an environment variable through does not, and neither does a
# Go or Rust binary reading one. One implementation, so a path handed to a
# tilde-ignorant tool and a path compared against another copy of itself agree.
#
# The tilde is assembled into a variable rather than written as a literal: the
# linter reads a quoted literal one as a tilde that failed to expand, and it
# would be right about every other use of one.
#
# An unset or empty HOME is a refusal rather than a silent expansion to `/`: the
# callers write the result into an environment variable or compare it against
# another path, and a plausible-looking `/.cache/...` is worse in both places
# than a stop.
expand_home() {
	local tilde='~'
	case $1 in
	"$tilde" | "${tilde}/"*)
		[ -n "${HOME:-}" ] ||
			die "cannot expand a leading tilde in ${1}: HOME is unset or empty"
		;;
	esac
	case $1 in
	"$tilde") printf '%s\n' "$HOME" ;;
	"${tilde}/"*) printf '%s\n' "${HOME}/${1#"${tilde}/"}" ;;
	*) printf '%s\n' "$1" ;;
	esac
}

# Fail with a headline and any number of indented detail lines.
die() {
	echo "${prog}: $1" >&2
	shift
	local line
	for line in "$@"; do
		echo "    ${line}" >&2
	done
	exit 1
}

# ---------------------------------------------------------------------------
# The device build
# ---------------------------------------------------------------------------

# ELF e_machine for AArch64. A platform flag that failed to take effect produces
# an x86_64 binary that runs perfectly on the workstation and not at all on the
# device.
elf_machine_aarch64=183

# Refuse unless the bazel the caller named ($bazel, REACHY_BAZEL's value) is
# there. Deliberately says what the Makefile's require-bazel target says: these
# scripts run outside make, and REACHY_BAZEL has no Makefile equivalent. The
# argument names what would have been built.
require_bazel() {
	local noun=$1
	command -v -- "$bazel" >/dev/null 2>&1 ||
		die "the ${noun} is built by bazel and ${bazel} is not installed." \
			"Install bazelisk; .bazelversion pins the Bazel release it fetches." \
			"Or point REACHY_BAZEL at the bazel to use."
}

# Refuse a binary that is not an aarch64 ELF. Run on Bazel's outputs before
# anything reaches a contract path, so the age a deploy script reads is never
# that of an artefact no check passed.
verify_aarch64() {
	local out=$1

	# e_machine — bytes 18 and 19 of the ELF header, little-endian. Read with
	# od so the check costs no tooling a workstation might not carry.
	local machine
	machine=$(od -An -tu1 -j18 -N2 -- "$out" | awk '{print $1 + $2 * 256}')
	[ "$machine" = "$elf_machine_aarch64" ] || die \
		"$(basename -- "$out") is an ELF for machine ${machine}, not AArch64 (${elf_machine_aarch64})." \
		"The platform flag did not take effect; the device cannot execute this."
}

# Several targets as one cquery set expression, so one question can name a whole
# payload's worth of outputs. Shared because both build paths — the device
# payload and the host run — ask exactly one such question per kind of answer,
# and a change to how targets are joined has to reach both.
union() {
	local expr=$1
	shift
	local target
	for target in "$@"; do
		expr="${expr} + ${target}"
	done
	echo "$expr"
}

# The two values a logger configuration has to restate.
#
# The launcher configs the compositions render carry no per-process arguments
# beyond a process description, so every process a run starts is flagless and
# finds its channel buffers under the compiled-in pinion defaults. A logger is
# handed those two values as configuration instead, and a logger looking
# somewhere else is a run that logs nothing while the gesture runs perfectly.
# Nothing at run time can notice that, so it is asserted before anything is
# staged -- on the device path and on the host run, whose own logger config
# carries the same two lines for the same reason.
pinion_defaults=(
	'pinion_shm_root: "/dev/shm"'
	'pinion_namespace: ""'
)

# The pinion agreement, checked against one logger configuration.
#
#   check_pinion_defaults <workspace-relative path>
#
# Line-oriented and exact, leading whitespace allowed, the same shape
# deploy-motion.sh reads a scalar with: what must hold is that the file states
# these values and not some others.
check_pinion_defaults() {
	local config=$1 want line
	[ -f "${repo_root}/${config}" ] ||
		die "the run wants ${config} and the tree has no such file."
	for want in "${pinion_defaults[@]}"; do
		line=$(sed -n "s/^[[:space:]]*\(${want%%:*}: .*\)\$/\1/p" \
			-- "${repo_root}/${config}" | head -n 1)
		[ "$line" = "$want" ] || die \
			"${config} states '${line:-nothing}' where the run needs '${want}'." \
			"Every process a run starts is flagless -- the rendered launcher config cannot" \
			"carry pinion flags -- so these two values are the compiled-in defaults, and the" \
			"logger has to name the same buffers the control process creates."
	done
}

# Where Bazel put what it built, as a listing of workspace-relative paths.
# cquery rather than the bazel-bin symlink: that symlink points at whatever
# configuration ran last, and a plain `bazel test //...` afterwards repoints it
# at the host one.
#
# The argument is a target or a cquery set expression, so one question can name
# a whole payload's worth of outputs. Callers pass $bazel and $build_flags: the
# cquery has to describe the configuration the build used, or it names files from
# some other one.
#
# An empty answer is a refusal rather than an empty listing a caller has to
# notice: every caller here asked because it wants to install something.
bazel_files() {
	local expr=$1 out
	out=$("$bazel" cquery "${build_flags[@]}" --output=files \
		--ui_event_filters=-info --noshow_progress -- "$expr") ||
		die "bazel cannot name the outputs of ${expr}, so there is nothing to install."
	[ -n "$out" ] ||
		die "bazel named no output file for ${expr}."
	echo "$out"
}

# A workspace-relative path from `bazel_files` as an absolute one, refusing if
# nothing is there.
#
# The listing's paths resolve through the bazel-out convenience symlink. That
# symlink is the one assumption these scripts make about the output tree, and
# .bazelrc.user can rename it (--symlink_prefix) or switch it off
# (--noexperimental_convenience_symlinks), so the refusal names both flags: the
# build succeeding and the path not resolving is that setting, not a build that
# produced nothing.
bazel_resolve() {
	local out=$1
	[ -f "${repo_root}/${out}" ] ||
		die "the build named ${out} and no file is there." \
			"If the build itself succeeded, the bazel-out convenience symlink is renamed" \
			"or disabled — check .bazelrc.user for --symlink_prefix or" \
			"--noexperimental_convenience_symlinks."
	echo "${repo_root}/${out}"
}

# One file out of a `bazel_files` listing, chosen by exact basename and resolved.
#
#   bazel_named_in <listing> <basename>
#
# Exact rather than a substring match: a listing carries the sources of what was
# built as well as its outputs, so a looser match can name a file nobody asked
# for. A basename the listing does not carry is a refusal — the compiler's output
# names changed, or something was renamed upstream.
bazel_named_in() {
	local listing=$1 want=$2 line out=
	while IFS= read -r line; do
		[ "$(basename -- "$line")" = "$want" ] || continue
		out=$line
		break
	done <<<"$listing"
	[ -n "$out" ] ||
		die "the build emits no ${want}." \
			"The compiler's output names changed, or the system's cpu domain was renamed."
	bazel_resolve "$out"
}

# ---------------------------------------------------------------------------
# The device deploy
# ---------------------------------------------------------------------------

# The two services on a unit that open the servo bus. brenn-app is the payload's
# own; reachy-motiond is the brenn-pod motion monolith. Either one holding the
# port turns a deploy into a run that meets a held bus, so both are refused, and
# both deploy scripts refuse them in the same words.
service=brenn-app.service
motiond_service=reachy-motiond.service

# The remote probe: two questions and this repo's two exit codes. Written once
# because the codes are a contract — docs/bench-runbook.md documents them as one
# thing — and because a caller appends its own work to this string so the
# question and the work are one ssh invocation. Asked separately, a service can
# start in between and the deploy lands beside it anyway.
bus_probe() {
	printf '%s; %s' \
		"systemctl is-active --quiet ${service} && exit 3" \
		"systemctl is-active --quiet ${motiond_service} && exit 4"
}

# Turn the probe's exit code into the refusal. 3 and 4 are bus_probe's own
# answers and nothing else produces them; 255 is ssh itself, which is why the
# probe does not signal by nonzero exit alone — that reads an unreachable host
# as a service being down. Every other code, 0 included, returns: what it means
# is the caller's, because the caller chose what it appended to the probe.
#
#   bus_refusal <rc> <what a run of this is> <what did not happen> [detail...]
#
# Callers set $host, as they do for ssh_root.
bus_refusal() {
	local rc=$1 run=$2 aftermath=$3
	shift 3
	case "$rc" in
	3)
		die "${service} is running on ${host}, and ${run} will not share the servo bus with it." \
			"Stop it, run the test, and start it again when you are done:" \
			"    ssh root@${host} systemctl stop ${service}"
		;;
	4)
		die "${motiond_service} is running on ${host}, and it holds the servo bus." \
			"Stop it, run the test, and start it again when you are done:" \
			"    ssh root@${host} systemctl stop ${motiond_service}" \
			"    ssh root@${host} systemctl start ${motiond_service}" \
			"(The port's own flock refuses either way; this is the message that says which process has it.)"
		;;
	255)
		die "ssh to root@${host} failed; ${aftermath}." "$@"
		;;
	esac
}

# ssh as root to the host the caller is deploying to. $host is the script's
# first argument, parsed before any call to this.
ssh_root() {
	ssh -o BatchMode=yes "root@${host}" "$@"
}

# Refuse an artefact built before the newest commit that touched the paths it is
# built out of.
#
#   refuse_if_stale <artefact> <noun> <rebuild cmd> <deliberately> <stale-ok cmd> <path>...
#
# The trap this closes cost a bench night: a build step was taken for a
# once-per-session one, and a run whose findings looked like the machine's was
# reading a binary several commits old. Commit time against file time catches
# exactly that — it does not catch uncommitted edits, which is why the make
# targets build first and are the entry points to prefer.
#
# The rebuild it prescribes is what clears it, always: both build scripts stamp
# every file they stage, because Bazel hands back an output it did not have to
# relink untouched and a commit that changes no linked code would otherwise
# leave a current artefact refused with no way through but the escape hatch.
#
# A tree with no history for those paths (a tarball, a shallow clone that
# excluded them) is not evidence of staleness, so it says what it could not
# decide and proceeds.
refuse_if_stale() {
	local artefact=$1 noun=$2 rebuild=$3 deliberately=$4 stale_ok=$5
	shift 5
	local commit built
	commit=$(git -C "$repo_root" log -1 --format=%ct -- "$@" 2>/dev/null) || commit=
	if [ -z "$commit" ]; then
		echo "${prog}: no commit history for the workspace here, so the ${noun}'s age is unknown" >&2
		return 0
	fi
	built=$(stat -c %Y -- "$artefact") ||
		die "cannot read the age of ${artefact}"
	[ "$built" -lt "$commit" ] || return 0
	die "the ${noun} is older than the newest commit to the workspace, so a run of it is not a run of this tree." \
		"Built $(date -d "@${built}" '+%Y-%m-%d %H:%M:%S'), newest commit $(date -d "@${commit}" '+%Y-%m-%d %H:%M:%S')." \
		"Rebuild it:" \
		"    ${rebuild}" \
		"or, to ${deliberately}:" \
		"    ${stale_ok}"
}

# ---------------------------------------------------------------------------
# A run's records
# ---------------------------------------------------------------------------

# The run directory the logger named for the instant it opened, and a refusal
# when there is nothing in it.
#
#   run_directory <log-root> <no-directory-hint> <no-records-hint>
#
# Echoes the newest directory directly under the log root that holds a
# non-empty `.olog`. Both failures are refusals rather than a report over
# nothing: an empty log root is the namespace-mismatch failure that survives a
# whole session unnoticed if anything downstream is willing to read zero
# records. The two hints are the caller's, because where a process's console
# output landed and which configuration file states the namespace differ
# between a host staging tree and a device fetch.
run_directory() {
	local logs=$1 no_directory=$2 no_records=$3
	local dir found
	dir=$(find "$logs" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)
	[ -n "$dir" ] || die \
		"the logger wrote no run directory under ${logs}." "$no_directory"
	found=$(find "$dir" -name '*.olog' -size +0 -print -quit)
	[ -n "$found" ] || die \
		"${dir} holds no non-empty .olog file." "$no_records"
	echo "$dir"
}

# The label of the analyzer that judges a run's records. One string, because
# both run harnesses invoke the same analyzer over their own fetched or staged
# log and a rename has to reach both.
report_target=//cogs:first_motion_report

# Judge a run's records, and let the analyzer's verdict be the caller's.
#
#   report_verdict <run directory> [extra analyzer arguments...]
#
# The analyzer is a host tool over a log that has stopped being written, so it
# builds in the default configuration whatever configuration the payload was
# built in: callers pass $bazel and $build_flags, and the device harness's
# $build_flags is deliberately empty for that reason. The extra arguments are
# the caller's — the host run reads a staged log with a jitter band, a device
# run reads hardware timestamps strictly — and the exit status is returned
# rather than judged, because it is the wrapper's verdict.
report_verdict() {
	local run_dir=$1
	shift
	"$bazel" run "${build_flags[@]}" -- "$report_target" "$@" "$run_dir"
}
