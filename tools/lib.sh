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

# The name the push's account of which build the payload is goes by, at the root
# of the payload and at the root of a run's log root. Part of the payload: it is
# written into the staged directory before the push, so the one rsync that
# delivers the payload delivers the stamp with it and a stamp on the unit
# describes the payload beside it by construction. The build stages that
# directory from scratch, so nothing stale is left there to push, and it lists
# this name among the payload members a credential must not land on. The copy in
# the log root comes home with the records under the fetch's own name, with no
# fetch-side logic to put it there. It is also what a pod deploy from brenn-pod's
# deploy-reachy-pod.sh reads to recognise a robot — a rename here is invisible
# there, so this name is pinned by a case in `deploy-motion.test.sh`.
#
# Here rather than in either script: the build lists it and the push writes it,
# and three spellings of one payload member is a rename that leaves the build's
# collision list matching nothing.
provenance_name=provenance.txt

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

# The two ways to set one knob, as detail lines for `die`.
#
#   knob_remedy <VAR> <placeholder> [make goal]
#
# Every knob these scripts read can be named for one invocation or once in the
# gitignored `.local/reachy.conf`, which make includes unconditionally. Written
# here rather than at each refusal because the `?=` spelling is load-bearing --
# it is what yields to both a command-line and an environment value -- and a
# convention spelled once cannot be half-changed.
knob_remedy() {
	local var=$1 placeholder=$2 goal=${3:-speech-run}
	printf '%s\n' \
		"Name it for this invocation:" \
		"    make ${goal} ${var}=${placeholder}" \
		"or once, in the gitignored .local/reachy.conf:" \
		"    ${var} ?= ${placeholder}"
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
# The payload members that come from outside this tree
# ---------------------------------------------------------------------------
#
# Named here rather than in the build script because two scripts need them: the
# build stages them, and the push asks whether either has changed since. The
# commit-time freshness question above cannot see either -- no commit to this
# workspace touches them -- so they get their own, and a knob spelled once
# cannot answer the two scripts differently.

# The brenn-pod checkout this repo reads two things out of: the audio device's
# prebuilt binary, staged into the payload, and the provisioning make target
# that writes the pod's half of the voice link onto the unit.
#
# One physical fact — where that repo is — gets one knob, so a workstation whose
# checkouts are not siblings says so once. `REACHY_POD_BINARY` still wins for the
# binary alone: it names a file rather than a repository, which is what an
# artifact copied out of a build somewhere else is.
# A relative value is relative to this repository's root, not to whatever
# directory the caller happened to be in: the Makefile's own default is spelled
# relatively and exported, and a script run by hand from a subdirectory must
# resolve it to the same checkout the recipes do.
brenn_pod_dir=${BRENN_POD_DIR:-../brenn-pod}
case $brenn_pod_dir in
/*) ;;
*) brenn_pod_dir=${repo_root}/${brenn_pod_dir} ;;
esac

# The audio device's binary, which this repo does not build.
#
# `reachy-pod` links libusb-1.0 and libasound2 and is compiled natively in
# brenn-pod's pinned arm64 container, against the same dated Debian archive the
# device image is bootstrapped from. Nothing in this tree's hermetic sysroot can
# produce it, and vendoring the sources would be a second copy of a binary
# brenn-pod already ships — so the payload takes the artifact that build leaves
# behind.
#
# The default is where a sibling checkout puts it, because that is how the two
# repos are worked on and an inner loop that needs a knob set on every invocation
# is a knob people set wrong. It is a default and not an assumption: a path that
# is not there is a refused build naming both the knob and the command in the
# other repo that produces the file, so a different layout says so once rather
# than staging something unexpected.
pod_binary=${REACHY_POD_BINARY:-${brenn_pod_dir}/firmware/target/reachy-pod/payload/reachy-pod}
# Whether an operator named the artifact rather than the checkout, which decides
# whether the checkout's revision says anything about the file being staged.
pod_binary_named=${REACHY_POD_BINARY:+named}

# The brenn-pod revision this tree links its surface crates from.
#
#   pinned_pod_rev <MODULE.bazel path>
#
# One reader, because two of them drift: the build's provenance line below and
# `tools/module-pins.test.sh` both ask this question of the same line, and the
# self-check holds this answer equal to its own independent parse of the file.
# An empty answer means the file states no `BRENN_POD_REV` in a form this reads
# -- a branch name, a short id, a reflowed assignment -- which every caller has
# to say out loud rather than treat as an absent pin.
pinned_pod_rev() {
	sed -n 's/^BRENN_POD_REV = "\([0-9a-f]*\)".*/\1/p' -- "$1" 2>/dev/null |
		head -n 1
}

# What a payload's two halves of brenn-pod are, said in one line.
#
#   pod_provenance
#
# Two independent sources of brenn-pod feed one speech run. `reachy_host` links
# speech-surface, speech-pipeline and brenn-bridge from the revision
# `MODULE.bazel` pins, and `reachy_pod` is staged from whatever the working tree
# at `brenn_pod_dir` last built. Nothing makes the two agree, and a payload that
# pairs a pinned surface with a pod binary from another revision fails on the
# unit as a handshake or a protocol mismatch — a long way from this build, and
# costing a device round-trip to diagnose.
#
# What can be measured here is narrow, and the line says only that. `HEAD` is
# where the checkout stands *now*; the staged file is whatever the last build in
# that tree left behind, and a pull or a pin bump moves the one without touching
# the other. So the line never calls a revision the binary's: it names the
# checkout's, and beside it the one comparison that bears on the file — the
# binary's mtime against the HEAD commit's date, the same arithmetic
# `refuse_if_stale` uses. A binary older than that commit was built at some
# earlier revision, whatever the checkout says today.
#
# `rev-parse HEAD` also answers from an enclosing repository when the directory
# is not a checkout root — an unpacked archive under a tracked `$HOME` would
# otherwise be reported at the revision of whatever tracks it — so the toplevel
# is checked first and anything else is "answers no revision".
#
# A note and never a refusal: a development checkout legitimately sits ahead of
# the pin, and the checkout may not be a git checkout at all. Every shape it
# cannot read says what it could not read rather than staying silent, because
# silence here reads as agreement.
pod_provenance() {
	local module pinned tree top committed built age
	module=${repo_root}/MODULE.bazel
	if [ ! -f "$module" ]; then
		echo "there is no MODULE.bazel at ${repo_root}, so the linked surface's revision is unknown"
		return
	fi
	pinned=$(pinned_pod_rev "$module")
	if [ -z "$pinned" ]; then
		echo "MODULE.bazel is here but states no BRENN_POD_REV this can read, so the linked surface's revision is unknown"
		return
	fi
	if [ -n "$pod_binary_named" ]; then
		echo "the linked surface is pinned at ${pinned:0:12}; the audio-device binary is the artifact REACHY_POD_BINARY names, whose revision this cannot ask"
		return
	fi
	tree=
	top=$(git -C "$brenn_pod_dir" rev-parse --show-toplevel 2>/dev/null) || top=
	if [ -n "$top" ] &&
		[ "$(cd -P -- "$top" 2>/dev/null && pwd)" = "$(cd -P -- "$brenn_pod_dir" 2>/dev/null && pwd)" ]; then
		tree=$(git -C "$brenn_pod_dir" rev-parse HEAD 2>/dev/null) || tree=
	fi
	if [ -z "$tree" ]; then
		echo "the linked surface is pinned at ${pinned:0:12}; ${brenn_pod_dir} answers no revision, so where the audio-device binary came from is unknown"
		return
	fi
	committed=$(git -C "$brenn_pod_dir" log -1 --format=%ct 2>/dev/null) || committed=
	built=$(stat -c %Y -- "$pod_binary" 2>/dev/null) || built=
	if [ -z "$committed" ] || [ -z "$built" ]; then
		age="when the audio-device binary was built cannot be compared against it"
	elif [ "$built" -lt "$committed" ]; then
		age="the audio-device binary is older than that commit, so it was built at an earlier one"
	else
		age="the audio-device binary was built after that commit"
	fi
	if [ "$tree" = "$pinned" ]; then
		echo "the linked surface is pinned at ${pinned:0:12} and the checkout is at that revision; ${age}"
	else
		echo "the linked surface is pinned at ${pinned:0:12} and the checkout is at ${tree:0:12}: two revisions of brenn-pod in one payload; ${age}"
	fi
}

# The voice pipeline's own configuration, which this repo does not contain and
# will not.
#
# It holds a site's STT and TTS endpoints, its bus server and token, and this
# unit's link keys, so every copy of it belongs to whoever runs the machine.
# There is no shipped default: a tree-resident one would either carry somebody's
# infrastructure into a public repository or name endpoints that answer nobody,
# and the launcher entry is not the place to discover which.
#
# So this is an optional payload member. Named, it must be there -- an operator
# who said where the configuration is and typed the path wrong wants to hear it
# now, not from a unit's console. Unnamed, the default is the gitignored
# `host/speech.toml` of the working tree, and its absence is not a refusal: the
# host starts either way and says which of the two it is, and a motion run and a
# bench night need no speech configuration at all.
#
# A relative value is taken as the caller typed it -- against their working
# directory, not against this repository's root, which is where `brenn_pod_dir`
# anchors one. The two differ in where a relative value can come from: that
# knob's own default is relative and exported by the Makefile, so a value
# arriving there may be one no human typed, and only the repository root makes
# it name the same checkout from every directory. This one's default is already
# absolute, so every relative value is one somebody typed at a prompt, and their
# prompt is what they typed it against.
speech_config=${REACHY_SPEECH_CONFIG:-${repo_root}/host/speech.toml}
# Whether an operator named it, which is the whole difference between "not there
# and that is the shipped state" and "not there and you asked for it".
speech_config_named=${REACHY_SPEECH_CONFIG:+named}

# Where the speech configuration goes under the payload root: the path
# `host/host_launch.textproto` spells in the host's `--speech-config` argument.
# The two have to agree, and `tools/build-motion.test.sh` holds them to each
# other.
speech_config_path=host/speech.toml

# The voice host's own configuration, which is a per-unit file and therefore not
# in this tree either.
#
# It names the machine the head answers to, so a copy of it belongs to whoever
# runs that machine. The tree carries `host/host_params.example.textproto`,
# which is the schema's worked example and is staged by nothing.
#
# Unlike the speech configuration this is not optional: the host cannot start
# without it, so an absent file -- named or defaulted -- is a refused build.
#
# The unnamed default is under `.local/` rather than `host/` deliberately: Bazel
# does not read `.gitignore`, so a gitignored operator file at
# `host/host_params.textproto` would be a file the `host/` package can see and a
# test could pick up. `.local/` is already every other operator file's home.
host_params=${REACHY_HOST_PARAMS:-${repo_root}/.local/host_params.textproto}

# Where it goes under the payload root: the host's own `DEFAULT_CONFIG`, which is
# why the launcher entry passes no `--config` at all.
host_params_path=host/host_params.textproto

# ---------------------------------------------------------------------------
# Reading the speech configuration
# ---------------------------------------------------------------------------
#
# The speech configuration names files — the pod's key table, the bus token —
# and endpoints, and two scripts have to know which: the build stages the files
# it names, and the push asks whether any of them has moved on since. The real
# loader is `speech-surface`'s and `reachy_host --check` runs it; what these
# scripts need is four string values out of a file they never write, so this is
# a reader and not a parser, and every shape it cannot read confidently is a
# refusal rather than a guess.

# One string value out of a TOML file, or empty when the file does not state it.
#
#   toml_table_value <file> <table> <key>
#
# The table is a header's name without its brackets (`brenn.bridge`), or the
# empty string for the keys above the first header. Scoping is what keeps a
# `url` under `[stt]` from answering for a `url` under `[tts]`.
#
# Keys and values may be bare, double-quoted or single-quoted, and a `#` inside
# a quoted value is part of the value: a path truncated at a `#` is a plausible
# wrong file, and a wrong file here is a credential the payload does not carry.
# Escapes inside a basic string are not decoded — nothing this reads holds one.
#
# Seven shapes are refusals, because each of them has a reading this would get
# wrong silently: a value whose quoting does not close; the same key stated
# twice in one table (which of the two the host loads is not this reader's to
# decide); a table header this cannot parse, which would file every key after it
# under the wrong name; an array-of-tables header, whose keys belong to a table
# no caller asks for; a dotted key inside the table being read, which states a
# nested table this scoping does not descend into; an inline table there, whose
# keys are on a line this reads as one value; and a multiline string anywhere in
# the file, whose body this would read as TOML of its own. The last four are the
# spellings that would otherwise read as *absent* — a staged payload with no
# credential in it and nothing said. The refusal names the remedy: these four
# keys want simple `key = "value"` spellings.
#
# A caller reads this through a command substitution, so the refusal's exit is
# the subshell's: `value=$(toml_table_value ...) || exit 1`.
toml_table_value() {
	local file=$1 table=$2 want=$3 out status=0
	local label="${table:+[${table}] }${want}"
	out=$(awk -v table="$table" -v want="$want" '
		# The line up to a comment marker outside quotes, or the whole line.
		function strip_comment(line,   i, c, q, n) {
			q = ""
			n = length(line)
			for (i = 1; i <= n; i++) {
				c = substr(line, i, 1)
				if (q != "") { if (c == q) q = "" }
				else if (c == "\"" || c == "'\''") q = c
				else if (c == "#") return substr(line, 1, i - 1)
			}
			return line
		}
		# Where the key ends and the value begins: the first = outside quotes.
		function eq_index(line,   i, c, q, n) {
			q = ""
			n = length(line)
			for (i = 1; i <= n; i++) {
				c = substr(line, i, 1)
				if (q != "") { if (c == q) q = "" }
				else if (c == "\"" || c == "'\''") q = c
				else if (c == "=") return i
			}
			return 0
		}
		function trim(s) { gsub(/^[[:space:]]+|[[:space:]]+$/, "", s); return s }
		# The contents of a quoted token, or the token itself. Both ends must
		# agree: a half-quoted token is malformed, not a value with a stray mark.
		function unquote(s,   c, n) {
			n = length(s)
			c = substr(s, 1, 1)
			if (c == "\"" || c == "'\''") {
				if (n >= 2 && substr(s, n, 1) == c)
					return substr(s, 2, n - 2)
				malformed = 1
				return s
			}
			return s
		}
		BEGIN {
			current = ""; hits = 0; value = ""; bad = 0
			# The two multiline openers, assembled rather than
			# written: three single quotes inside a shell-quoted
			# program are unreadable at every later edit.
			three_basic = "\"\"\""
			three_literal = sprintf("%c%c%c", 39, 39, 39)
		}
		{
			line = strip_comment($0)
			head = trim(line)
			if (substr(head, 1, 2) == "[[") { bad = 5; exit }
			if (substr(head, 1, 1) == "[") {
				if (substr(head, length(head), 1) != "]") { bad = 4; exit }
				current = trim(substr(head, 2, length(head) - 2))
				gsub(/^["'\'']|["'\'']$/, "", current)
				next
			}
			eq = eq_index(line)
			if (eq == 0) next
			# A multiline string, in any table: this reads one line
			# per value, so the body of one is read as TOML -- a
			# bracketed line in it re-scopes every key after it and
			# the keys wanted here go absent with nothing said.
			# Refused wherever it is, because the damage is to the
			# scope this walk keeps rather than to the value.
			opener = trim(substr(line, eq + 1))
			if (substr(opener, 1, 3) == three_basic ||
			    substr(opener, 1, 3) == three_literal) { bad = 8; exit }
			if (current != table) next
			malformed = 0
			raw_key = trim(substr(line, 1, eq - 1))
			raw_val = trim(substr(line, eq + 1))
			key = unquote(raw_key)
			val = unquote(raw_val)
			# A dotted key states a nested table and an inline table
			# holds its keys on this one line: both are in the scope
			# being read and neither is descended into, so a key
			# spelled either way would read as absent.
			if (raw_key == key && index(raw_key, ".") > 0) { bad = 6; exit }
			if (substr(raw_val, 1, 1) == "{") { bad = 7; exit }
			if (key != want) next
			if (malformed) { bad = 2; exit }
			hits++
			if (hits > 1) { bad = 3; exit }
			value = val
		}
		END {
			if (bad) exit bad
			if (hits == 1) print value
		}
	' "$file") || status=$?
	case "$status" in
	0) ;;
	2)
		die "cannot read ${label} from ${file}: the value's quoting does not close." \
			"This reader takes a simple 'key = \"value\"' spelling and refuses the rest;" \
			"it will not guess where a credential path ends."
		;;
	3)
		die "${file} states ${label} twice, and which one the host would load is not this reader's to decide." \
			"Leave one of them."
		;;
	4)
		die "${file} has a table header this reader cannot parse." \
			"Every key after it would be filed under the wrong table, so nothing is read."
		;;
	5)
		die "${file} has an array-of-tables header, and this reader files keys under one table name." \
			"Reading ${label} past it would answer from a table nothing asked for." \
			"These four keys want plain '[table]' headers and simple 'key = \"value\"' lines."
		;;
	6)
		die "${file} states a dotted key in the ${table:-top-level} table, which names a nested table this reader does not descend into." \
			"Spelling ${label} that way would read as absent and stage no credential." \
			"Write the nested table as its own '[table]' header instead."
		;;
	7)
		die "${file} states an inline table in the ${table:-top-level} table, whose keys are all on one line." \
			"Spelling ${label} that way would read as absent and stage no credential." \
			"Write it as its own '[table]' header with simple 'key = \"value\"' lines."
		;;
	8)
		die "${file} states a multiline string, and this reader takes one line per value." \
			"Its body would be read as TOML: a bracketed line inside it files every key" \
			"after it under a table nothing asked for, and ${label} would read as absent." \
			"Write the multiline values in this file as single-line strings."
		;;
	*)
		die "cannot read ${label} from ${file}."
		;;
	esac
	printf '%s\n' "$out"
}

# The speech configuration's credential path fields, as `<table>\t<key>`.
#
# Both of them are optional and each is optional for its own reason: a
# configuration with no `[brenn.bridge]` is a voiced, bus-less pipeline, which
# is legal and stages no token.
speech_credential_keys=(
	$'\tpod_psk_file'
	$'brenn.bridge\ttoken_file'
)

# Refuse a credential path the payload cannot carry.
#
#   check_credential_path <config> <key> <value>
#
# Every file the configuration names travels inside the payload and is named by
# the payload-relative path it will occupy — which is also the path the host
# resolves at run time, because the launcher starts it with the payload root as
# its working directory. An absolute path is the workstation-era spelling: it would resolve on
# the machine the payload was built on and name nothing on the unit.
check_credential_path() {
	local config=$1 key=$2 value=$3
	case "$value" in
	/*)
		die "${key} in ${config} is the absolute path ${value}, and the payload carries its own credentials." \
			"Name it relative to the payload root, with the file beside the configuration:" \
			"a re-push then replaces it and the freshness check covers it. An absolute path" \
			"resolves on this machine and names nothing on the unit."
		;;
	esac
	case "/${value}/" in
	*/../*)
		die "${key} in ${config} is ${value}, which climbs out of the payload." \
			"The file is staged at that path under the payload root and the host resolves it" \
			"there; a path leaving the payload is a file no push carries."
		;;
	esac
	case "$value" in
	*/ | "")
		die "${key} in ${config} is '${value}', which names no file."
		;;
	esac
	# A `.` component or a doubled slash names the same file under a spelling
	# nothing else uses. The collision check the build runs is textual --
	# `./robotcpu.textproto` matches no member's name and installs over the
	# member all the same, because the filesystem resolves what the compare
	# did not. So the path has to name where it points to.
	case "/${value}/" in
	*/./* | *//*)
		die "${key} in ${config} is ${value}, which carries a . or an empty component." \
			"The payload's members are compared against this path by name, so a spelling" \
			"that resolves to one of them without matching it would install a credential" \
			"over a launcher config or a model. Name the file plainly."
		;;
	esac
}

# The credential files a speech configuration names, as
# `<key>\t<payload-relative path>\t<source path>` lines, one per key it states.
#
#   speech_credential_paths <config>
#
# A configuration that is not there names nothing: a payload built without one
# is the ordinary case, and the build refuses a *named* one that is missing
# before this is ever asked. Every value that is there is checked here, so the
# build and the push cannot disagree about which paths the payload's credentials
# occupy.
#
# The source column is the other half of the assembly-directory convention: the
# configuration names the path a file will occupy in the payload, and the file
# itself sits beside the configuration under that same name. Emitted here rather
# than re-joined at each call site, so the build's staging and the push's
# freshness check cannot come to disagree about where a credential came from —
# which would be a stale secret shipped under a green verdict.
#
# Read through a command substitution, as `toml_table_value` is.
speech_credential_paths() {
	local config=$1 entry table key value
	[ -f "$config" ] || return 0
	for entry in "${speech_credential_keys[@]}"; do
		table=${entry%%$'\t'*}
		key=${entry#*$'\t'}
		value=$(toml_table_value "$config" "$table" "$key") || exit 1
		[ -n "$value" ] || continue
		check_credential_path "$config" "$key" "$value"
		printf '%s\t%s\t%s\n' "$key" "$value" "$(dirname -- "$config")/${value}"
	done
}

# The speech configuration's service endpoints, as `<table>\t<key>`.
#
# Both are optional for the same reason the credential fields are: a
# configuration naming neither is a pipeline with no speech services, and a
# table that is not there is asked nothing.
speech_service_keys=(
	$'stt\turl'
	$'tts\turl'
)

# Refuse a service URL a remote command cannot safely carry.
#
#   check_service_url <config> <table> <value>
#
# The value is pasted into a command run on the unit as root, so what is
# accepted is the shape of a URL and nothing else: a scheme this repo speaks and
# a rest made of characters that mean the same thing to every shell between here
# and there. A value that is something other than a URL is a refusal where it is
# read rather than a quoting that has to hold at three sites.
check_service_url() {
	local config=$1 table=$2 value=$3
	case "$value" in
	http://?* | https://?*) ;;
	*)
		die "[${table}] url in ${config} is '${value}', which is not an http or https URL." \
			"The speech run asks the unit to reach that address before it starts anything," \
			"so the value has to be one a URL fetch can be pointed at."
		;;
	esac
	case "$value" in
	*[!A-Za-z0-9:/._~%-]*)
		die "[${table}] url in ${config} is '${value}', which carries a character this cannot pass on." \
			"That URL is pasted into a command run on the unit as root, so only" \
			"[A-Za-z0-9:/._~%-] is accepted here — a service endpoint is a scheme, a host," \
			"a port and a path, and nothing that means something to a shell."
		;;
	esac
}

# The speech services a configuration names, as `<table>\t<url>` lines, one per
# table it states.
#
#   speech_service_urls <config>
#
# What the speech run's on-unit reachability preflight is built from: the
# vantage that decides whether a pipeline can hear and speak is the robot's, and
# a workstation-era `localhost` endpoint is the migration error that looks like
# a deaf machine. A configuration that is not there names nothing.
#
# Trailing slashes come off here, once, because the caller appends a probe path:
# a configured `http://host:8000/` would otherwise be asked for `//v1/models`,
# and the 404 that comes back would be read as an address the robot cannot
# reach.
#
# Read through a command substitution, as `toml_table_value` is.
speech_service_urls() {
	local config=$1 entry table key value
	[ -f "$config" ] || return 0
	for entry in "${speech_service_keys[@]}"; do
		table=${entry%%$'\t'*}
		key=${entry#*$'\t'}
		value=$(toml_table_value "$config" "$table" "$key") || exit 1
		[ -n "$value" ] || continue
		check_service_url "$config" "$table" "$value"
		while [ "${value%/}" != "$value" ]; do
			value=${value%/}
		done
		printf '%s\t%s\n' "$table" "$value"
	done
}

# Refuse a payload member whose out-of-tree source has moved on since the build
# staged it.
#
#   refuse_if_source_newer <staged> <source> <noun> <stale-ok cmd>
#
# The commit-time check above answers for everything this tree builds and for
# nothing else, and the members staged from outside the tree are the everything
# else: a rebuilt `reachy-pod`, a speech configuration whose token or link keys
# were just rotated, or the operator's host parameters, changes nothing any
# commit to this workspace can date. Pushed without a rebuild, that is the
# previous binary, the previous credentials or the previous unit's identity
# shipped under a green freshness verdict — the exact mistake the age check
# exists to stop, and the credential flavour of it is the one an operator would
# chase on the unit.
#
# A source that is not there says nothing: the pod binary's absence is refused by
# the build, a payload built with no speech configuration is the ordinary case,
# and a host parameters file that is nowhere is refused by the build too. A
# source that is there against a payload carrying no copy is a refusal for the
# same reason a newer one is — the file existed nowhere when this payload was
# staged.
refuse_if_source_newer() {
	local staged=$1 source=$2 noun=$3 stale_ok=$4
	[ -f "$source" ] || return 0
	local source_at staged_at
	source_at=$(stat -c %Y -- "$source") ||
		die "cannot read the age of ${source}"
	if [ ! -f "$staged" ]; then
		die "there is a ${noun} at ${source} and the staged payload carries none, so this push would ship a unit without it." \
			"Rebuild it:" \
			"    make motion-build" \
			"or, to push the payload as it stands:" \
			"    ${stale_ok}"
	fi
	staged_at=$(stat -c %Y -- "$staged") ||
		die "cannot read the age of ${staged}"
	[ "$staged_at" -lt "$source_at" ] || return 0
	die "the ${noun} at ${source} is newer than the copy in the payload, so this push would ship the older one." \
		"Staged $(date -d "@${staged_at}" '+%Y-%m-%d %H:%M:%S'), source $(date -d "@${source_at}" '+%Y-%m-%d %H:%M:%S')." \
		"Rebuild it:" \
		"    make motion-build" \
		"or, to push the copy the payload already carries:" \
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

# The analyzer of a speech run, which reads both sides of a fetch: what a
# supervised session holds depends on what a person said to the robot, so the
# pipeline's own narration is the evidence for what was asked and the records
# are the evidence for what the head did with it. The motion analyzer's grid
# arithmetic still has nothing to say about a session nobody budgeted.
speech_report_target=//cogs:speech_run_report

# Judge a speech run's fetch, and let the analyzer's verdict be the caller's.
#
#   speech_verdict <fetched records directory>
#
# The argument is the fetch's own directory, not a run directory inside it: the
# analyzer names the console beside it from that spelling and finds the run
# directory within it the way `run_directory` above does. Host tool over a
# stopped log, so it builds in the default configuration, like `report_verdict`.
speech_verdict() {
	"$bazel" run "${build_flags[@]}" -- "$speech_report_target" "$1"
}
