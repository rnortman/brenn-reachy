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
# It also reads six variables the sourcing script owns and it cannot assign
# itself: `bazel` (which bazel to run, REACHY_BAZEL's value), `build_flags` (the
# array naming the configuration a build and its cqueries share), `host` (the
# device being deployed to, the script's first argument), `payload` (the staged
# payload directory), `experiment_dir` (the overlay laid over it, or empty) and
# `staged_wake_models` (the site-supplied models the staged speech configuration
# names). Each is named in the doc of every function that reads it.
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
# Line-oriented and exact, leading whitespace allowed, the same line shape
# `textproto_string` reads: what must hold is that the file states
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
# binary, which the payload build compiles there and stages, and the writer that
# composes the pod's half of the voice link into the payload
# (`firmware/tools/provision-reachy-pod.sh`).
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

# The audio device's binary, which this repo does not compile.
#
# `reachy-pod` links libusb-1.0 and libasound2 and is compiled natively in
# brenn-pod's pinned arm64 container, against the same dated Debian archive the
# device image is bootstrapped from. Nothing in this tree's hermetic sysroot can
# produce it, so the payload takes what that container leaves behind — but the
# build in this tree runs it. The payload output of a sibling checkout is that
# container's path, which is why the default is spelled as one.
#
# `REACHY_POD_BINARY` names a file rather than a checkout, and that is the escape
# hatch for an artifact copied out of a build elsewhere: named, the build stages
# it and checks nothing about where it came from.
pod_binary=${REACHY_POD_BINARY:-${brenn_pod_dir}/firmware/target/reachy-pod/payload/reachy-pod}
# Whether an operator named the artifact rather than the checkout, which decides
# whether the checkout's revision says anything about the file being staged.
pod_binary_named=${REACHY_POD_BINARY:+named}
# The build script in the other repo, and the record it writes beside the binary
# it produced: `commit=`, `dirty=` and `sha256=` of that binary. Spelled here
# because two things read the pair — the build that runs the script and the
# provenance line that reports what the check concluded.
pod_build_script=${brenn_pod_dir}/firmware/tools/build-reachy-pod.sh
pod_sidecar=${brenn_pod_dir}/firmware/target/reachy-pod/payload/reachy-pod.build

# What the pod half of the payload turned out to be, filled in by
# `refuse_unless_pod_checkout_matches` below and read by the build's stamp and
# its provenance line. Empty until the check has run, which is the state the
# named-artifact case stays in.
pod_commit=
pod_dirty=
pod_source_line=

# The brenn-pod revision this tree links its surface crates from.
#
#   pinned_pod_rev <MODULE.bazel path>
#
# One reader, because two of them drift: the build's provenance line below and
# `tools/module-pins.sh` both ask this question of the same line, and the pin
# check holds this answer equal to its own independent parse of the file.
# An empty answer means the file states no `BRENN_POD_REV` in a form this reads
# -- a branch name, a short id, a reflowed assignment -- which every caller has
# to say out loud rather than treat as an absent pin.
pinned_pod_rev() {
	sed -n 's/^BRENN_POD_REV = "\([0-9a-f]*\)".*/\1/p' -- "$1" 2>/dev/null |
		head -n 1
}

# The packages this tree takes from brenn-pod, space-padded for a substring
# test. Named here rather than derived from the file, so a spec that quietly
# loses its remote is a missing spec rather than a package this stops looking
# for. `tools/module-pins.sh` keeps its own copy of the list and holds the two
# equal, so a package added on one side alone is a failed case.
pod_crate_packages=" speech-surface speech-pipeline brenn-bridge pod-ingest "

# The working trees the `crate.spec`s resolve from, when they do.
#
#   pod_overlay_paths <MODULE.bazel path>
#
# The overlay form the developing-a-seam block above `BRENN_POD_REV` describes:
# a spec's `git`/`rev` pair replaced by `path = "../brenn-pod/<crate path>"`.
# An overlaid tree keeps its `BRENN_POD_REV` line, so the pin alone would name a
# revision the binaries did not come from; this is the question that tells the
# two apart, and it is asked first.
#
# Every uncommented `path =` inside a `crate.spec(` … `)` block whose `package`
# is one of brenn-pod's, comma-separated in the file's own order, and empty when
# none of them names one. All of them and not the first alone, because one
# containment check over one path would pass a file whose four specs name two
# different working trees — the same two-revisions payload the mixed form
# reaches, spelled entirely in overlays. A line
# reader over the literals a person typed, like the self-check's: commented text
# is not a spec attribute, and a `path =` outside a spec belongs to some other
# dependency's override. The package decides, because most of the twenty specs
# in the file are somebody else's crate: an overlaid `nalgebra` says nothing
# about which brenn-pod the voice host was built from, and a field that named it
# would hide the pin that did build it.
#
# Every brenn-pod spec is read, and not only up to the first overlaid one,
# because the file has to say one thing: the overlay form is all four specs at
# once, and a file where some are overlaid and the rest are on the pin links two
# revisions of brenn-pod into one voice host -- the state the refusals below
# exist to make unreachable, arriving through the file rather than through the
# checkout. Status 3 is that refusal, and the printed answer is the two sides of
# it, `<overlaid packages>|<pinned packages>`, comma-separated, for the message
# that reports it.
#
# The whole block is read to its closing paren before it answers, so the
# attribute order inside a spec cannot hide either key.
#
# The block form -- `crate.spec(` alone on its line, one attribute per line --
# is a precondition, and a spec written any other way is refused rather than
# read past: a compact `crate.spec(package = "…", path = "…")` would otherwise
# be invisible here and the payload would be stamped with the pin that did not
# build it, which reads as authoritative where `unknown` reads as "cannot tell".
# Status 2 is that refusal; the printed answer stays empty.
pod_overlay_paths() {
	awk -v packages="$pod_crate_packages" -- '
		/^[ \t]*#/ { next }
		/crate\.spec\(/ && !/^crate\.spec\($/ { compact = 1; exit 2 }
		/^crate\.spec\($/ { inspec = 1; pkg = ""; path = ""; next }
		inspec && /^\)/ {
			if (index(packages, " " pkg " ")) {
				if (path != "") {
					overlaid = overlaid (overlaid == "" ? "" : ",") pkg
					paths = paths (paths == "" ? "" : ",") path
				} else {
					pinned = pinned (pinned == "" ? "" : ",") pkg
				}
			}
			inspec = 0
			next
		}
		inspec && /^[ \t]*path[ \t]*=[ \t]*"/ {
			path = $0
			sub(/^[^"]*"/, "", path)
			sub(/".*/, "", path)
		}
		inspec && /^[ \t]*package[ \t]*=[ \t]*"/ {
			pkg = $0
			sub(/^[^"]*"/, "", pkg)
			sub(/".*/, "", pkg)
		}
		END {
			if (compact) exit 2
			if (overlaid != "" && pinned != "") {
				print overlaid "|" pinned
				exit 3
			}
			if (paths != "") print paths
		}
	' "$1" 2>/dev/null
}

# Which brenn-pod the payload's voice host was built from, as one field value.
#
#   pod_build_source <MODULE.bazel path>
#
# `overlay:<path>` when every brenn-pod spec resolves from a working tree — the
# first spec's path, which is the surface's and which names the tree for a
# reader; `refuse_unless_pod_checkout_matches` is what holds the other three to
# the same tree — the
# pinned revision when they all resolve from the remote, and `unknown` when the
# file is absent, when it states a pin in a form `pinned_pod_rev` cannot read,
# when it writes a `crate.spec` in a form `pod_overlay_paths` refuses, or when its
# brenn-pod specs disagree about which of the two they are. The overlay is checked
# first because an overlaid tree still carries the pin line, and its refusal is
# taken as the whole answer: a file whose specs cannot be read cannot be said to
# resolve from the remote either.
#
# Two readers, for two questions. `refuse_unless_pod_checkout_matches` below
# takes this as the revision the pod has to be built from, and the build stamp
# records it as the `brenn_pod=` field, where it survives into a run's records:
# an operator reading a fetched run months later has nothing else to tell a
# pre-fix payload from a post-fix one.
pod_build_source() {
	local module=$1 overlay pinned
	[ -f "$module" ] || {
		echo unknown
		return
	}
	overlay=$(pod_overlay_paths "$module") || {
		echo unknown
		return
	}
	if [ -n "$overlay" ]; then
		echo "overlay:${overlay%%,*}"
		return
	fi
	pinned=$(pinned_pod_rev "$module")
	echo "${pinned:-unknown}"
}

# Refuse a payload whose two halves of brenn-pod would disagree.
#
#   refuse_unless_pod_checkout_matches <MODULE.bazel path>
#
# Two independent sources of brenn-pod feed one speech run. `reachy_host` links
# speech-surface, speech-pipeline and brenn-bridge from what `MODULE.bazel`
# resolves — a pinned revision, or a working tree under the overlay form the
# pin's comment block describes — and `reachy_pod` is compiled from the checkout
# at `brenn_pod_dir`. A payload that pairs a pinned surface with a pod built from
# another revision fails on the unit as a handshake or a protocol mismatch, a
# long way from this build and a device round-trip to diagnose. That is a state a
# payload should not be able to reach silently, so this is a refusal. A directory
# that is not a git checkout at all is an artifact, and `REACHY_POD_BINARY` is
# how an artifact is named.
#
# Pinned: the checkout must stand at the pin with a clean tree, because anything
# else names two revisions. Overlay: the checkout must be the tree the overlay
# resolves into, and a dirty tree is allowed — an overlaid tree is a working tree
# and the host links that same working tree, uncommitted edits included. Every
# overlaid spec is held to that one tree and not the first alone: four specs
# naming two working trees put two revisions inside the host itself, which is
# the mixed form's bug with an overlay on both sides of it.
#
# Mixed: a `MODULE.bazel` whose brenn-pod specs are half overlaid and half on the
# pin links two revisions into the host itself, before the pod is even built, so
# it is refused ahead of both — and named, because the four specs are hand-edited
# and a half-finished edit is what this shape is.
#
# Dirty is tracked edits only (`--untracked-files=no`). An untracked file is not
# a revision the pod could have been built from, and the overlay form this repo
# documents *generates* untracked files in that checkout — a `path =` spec makes
# bazel write a `BUILD.bazel` into every crate directory it reaches — so counting
# them would refuse every pinned build after an overlay round over litter the
# message could not explain. brenn-pod's `build-reachy-pod.sh` has to write its
# `dirty=` by the same definition, and `refuse_unless_pod_sidecar_agrees` below
# is what holds the two equal: they part, and every build refuses.
#
# `rev-parse HEAD` also answers from an enclosing repository when the directory
# is not a checkout root — an unpacked archive under a tracked `$HOME` would
# otherwise be reported at the revision of whatever tracks it — so the toplevel
# is checked first and anything else is not a checkout.
#
# Sets `pod_commit`, `pod_dirty` and `pod_source_line` for the stamp and the
# provenance report; prints nothing.
refuse_unless_pod_checkout_matches() {
	local module=$1 source overlay top head status overlay_abs pod_abs forms mix dirty_mark
	source=$(pod_build_source "$module")
	if [ "$source" = unknown ]; then
		mix=0
		forms=$(pod_overlay_paths "$module") || mix=$?
		if [ "$mix" = 3 ]; then
			die "${module} overlays some of its brenn-pod crates and pins the rest, so the voice host itself would link two revisions of brenn-pod." \
				"Overlaid from the working tree: ${forms%%|*}." \
				"Resolved from the pinned revision: ${forms#*|}." \
				"The overlay is all of them at once or none of them: put the rest on" \
				"path = \"../brenn-pod/<crate path>\", or put these back on git = BRENN_POD_GIT," \
				"rev = BRENN_POD_REV. The comment block above BRENN_POD_REV has both forms."
		fi
		die "this tree cannot say which brenn-pod its voice host links." \
			"${module} is absent, states no BRENN_POD_REV in a readable form, or writes a" \
			"crate.spec in a form tools/lib.sh refuses to read. The pod is built from" \
			"${brenn_pod_dir} and has to be built from the revision the surface links, so" \
			"there is nothing to check it against." \
			"Fix the pin, or name a prebuilt artifact with REACHY_POD_BINARY."
	fi
	top=$(git -C "$brenn_pod_dir" rev-parse --show-toplevel 2>/dev/null) || top=
	if [ -z "$top" ] ||
		[ "$(cd -P -- "$top" 2>/dev/null && pwd)" != "$(cd -P -- "$brenn_pod_dir" 2>/dev/null && pwd)" ]; then
		die "${brenn_pod_dir} is not a brenn-pod checkout, so the audio-device binary cannot be built or attributed." \
			"The payload's pod is compiled from that checkout and has to match the revision" \
			"the voice host links (${source})." \
			"BRENN_POD_DIR names the checkout, and REACHY_POD_BINARY names a bare artifact" \
			"copied out of a build elsewhere, which skips this check."
	fi
	head=$(git -C "$brenn_pod_dir" rev-parse HEAD 2>/dev/null) ||
		die "${brenn_pod_dir} answers no revision, so an audio-device binary built there could not be attributed to one." \
			"A checkout with no history is an artifact, and REACHY_POD_BINARY is how an" \
			"artifact is named; BRENN_POD_DIR names a checkout with history."
	status=$(git -C "$brenn_pod_dir" status --porcelain --untracked-files=no 2>/dev/null) ||
		die "${brenn_pod_dir} will not say whether it is dirty, so what an audio-device binary built there came from is unknown." \
			"REACHY_POD_BINARY names a prebuilt artifact and skips this check."
	case $source in
	overlay:*)
		overlay=${source#overlay:}
		pod_abs=$(realpath -m -- "$brenn_pod_dir" 2>/dev/null) || pod_abs=$brenn_pod_dir
		local spec_path
		# Each of them, so a file whose specs name two working trees is refused
		# by the second one rather than passed by the first.
		while IFS= read -r spec_path; do
			[ -n "$spec_path" ] || continue
			overlay_abs=$(realpath -m -- "${repo_root}/${spec_path}" 2>/dev/null) ||
				overlay_abs="${repo_root}/${spec_path}"
			case "${overlay_abs}/" in
			"${pod_abs}/"*) continue ;;
			esac
			die "the voice host links its speech crates from the working tree at ${spec_path}, and the audio-device binary would be built from ${brenn_pod_dir}." \
				"Two trees of brenn-pod in one payload is the state this check exists to stop." \
				"Point BRENN_POD_DIR at the overlaid tree, or overlay the checkout this build" \
				"is pointed at."
		done < <(pod_overlay_paths "$module" | tr ',' '\n')
		# The mark is a variable and not a command substitution inside the
		# string: a substitution whose last command is a failed test makes the
		# whole assignment exit non-zero, and under `set -e` that ends the build
		# with no message at all — on the clean-tree case, which is the common
		# one.
		dirty_mark=
		[ -z "$status" ] || dirty_mark="+dirty"
		pod_source_line="the voice host links the working tree at ${overlay} and the audio-device binary is built from ${brenn_pod_dir} at ${head:0:12}${dirty_mark}"
		;;
	*)
		if [ "$head" != "$source" ]; then
			die "the voice host links brenn-pod at ${source:0:12} and ${brenn_pod_dir} stands at ${head:0:12}, so this payload would carry two revisions of brenn-pod." \
				"Either check the pin out, so the pod is built from the revision the surface links:" \
				"    git -C ${brenn_pod_dir} checkout ${source}" \
				"or overlay that working tree in MODULE.bazel, so the host links it too — the" \
				"overlay form is in the comment block above BRENN_POD_REV." \
				"REACHY_POD_BINARY names a prebuilt artifact and skips this check."
		fi
		if [ -n "$status" ]; then
			die "${brenn_pod_dir} stands at the pinned revision ${source:0:12} with uncommitted changes, so the pod would be built from something no revision names while the voice host links the pin." \
				"Either commit or stash them:" \
				"    git -C ${brenn_pod_dir} status" \
				"or overlay that working tree in MODULE.bazel, so the host links it too — the" \
				"overlay form is in the comment block above BRENN_POD_REV."
		fi
		pod_source_line="the voice host links brenn-pod at ${source:0:12} and the audio-device binary is built from ${brenn_pod_dir} at that revision"
		;;
	esac
	pod_commit=$head
	if [ -n "$status" ]; then
		pod_dirty=true
	else
		pod_dirty=false
	fi
}

# One `key=value` out of the record the other repo's build writes.
#
#   pod_sidecar_field <sidecar path> <key>
#
# The first assignment of that key, and empty for a key the file does not state
# or a file that is not there — every caller below distinguishes those two by
# asking about the file first. Plain `key=value`, the form `audio.conf` and
# `provenance.txt` already use, read line-wise so a value containing `=` is the
# value and not a parse.
pod_sidecar_field() {
	sed -n "s/^$2=//p" -- "$1" 2>/dev/null | head -n 1
}

# Refuse a pod binary the other repo's build did not just produce from the tree
# this build checked.
#
#   refuse_unless_pod_sidecar_agrees <sidecar> <commit> <dirty> <staged file>
#
# The checkout check above reads the tree before the container build; this reads
# what the container build says it did, which is the same question asked of the
# artifact rather than of the directory. Two ways they part: the tree moved under
# the build — a checkout switched, a file saved, between the check and the
# container finishing — and the binary was replaced by hand afterwards, which the
# digest is the only witness to.
#
# No sidecar at all is a brenn-pod checkout predating the build script that
# writes one, which is a real state of a real checkout and so gets its own
# sentence rather than being reported as a disagreement.
refuse_unless_pod_sidecar_agrees() {
	local sidecar=$1 commit=$2 dirty=$3 staged=$4 said_commit said_dirty
	[ -f "$sidecar" ] ||
		die "the brenn-pod build left no record at ${sidecar}, so what it built cannot be checked against the checkout." \
			"A checkout whose firmware/tools/build-reachy-pod.sh predates that record is too" \
			"old for this build: pull brenn-pod, or name a prebuilt artifact with" \
			"REACHY_POD_BINARY, which skips this check."
	said_commit=$(pod_sidecar_field "$sidecar" commit)
	said_dirty=$(pod_sidecar_field "$sidecar" dirty)
	if [ "$said_commit" != "$commit" ] || [ "$said_dirty" != "$dirty" ]; then
		die "the brenn-pod build says it built ${said_commit:-no revision} (dirty=${said_dirty:-unknown}) and the checkout this build read stands at ${commit} (dirty=${dirty})." \
			"The tree moved while the container was building, so the binary is not the one" \
			"this payload was checked for. Build again with the checkout still."
	fi
	refuse_unless_pod_digest_matches "$sidecar" "$staged"
}

# The digest half of that record, asked of one file.
#
#   refuse_unless_pod_digest_matches <sidecar> <file>
#
# Asked twice, of two files. Of the build's own output, beside the revision
# check above, so a binary that does not match its record is refused before an
# arm64 build's worth of further work; and of the copy in the staging directory,
# which is the file the payload actually carries. Between those two there is a
# window — the machine sweep, the plan, the credentials — and a brenn-pod build
# run by hand in another terminal lands in it, replacing the file after it was
# checked and before it was copied. The staged copy is what the stamp and
# `provenance.txt` claim a revision for, so it is the one that has to be asked.
refuse_unless_pod_digest_matches() {
	local sidecar=$1 file=$2 said_sha sha
	said_sha=$(pod_sidecar_field "$sidecar" sha256)
	sha=$(sha256sum -- "$file" | cut -d' ' -f1) ||
		die "cannot digest ${file}, so what the brenn-pod build produced cannot be checked."
	[ "$said_sha" = "$sha" ] ||
		die "the brenn-pod build recorded ${said_sha:-no digest} for the audio-device binary and ${file} digests ${sha}." \
			"The file was replaced after that build wrote it, so what would be staged is not" \
			"what the record describes. Build brenn-pod again, or name the artifact with" \
			"REACHY_POD_BINARY, which stages a file with no record and says so."
}

# What a payload's two halves of brenn-pod are, said in one line.
#
#   pod_provenance
#
# The refusals above are what makes the two halves agree; this is the line that
# says which agreement it is, on every build, because a fetched run months later
# is read by somebody who has only the records. Three shapes, one per outcome the
# check can reach: the checkout at the pin, the checkout overlaid at some
# revision and possibly dirty, and an artifact named by the knob whose revision
# nothing here can ask.
#
# Every line this prints describes a payload whose halves were checked and agree.
pod_provenance() {
	if [ -n "$pod_binary_named" ]; then
		echo "the audio-device binary is the artifact REACHY_POD_BINARY names at ${pod_binary}, whose revision this cannot ask; the voice host links $(pod_build_source "${repo_root}/MODULE.bazel")"
		return
	fi
	# No line to print means the check never ran, and this function's promise
	# above is the one thing it cannot report: an unnamed pod whose halves
	# nobody compared is exactly the payload the refusals above exist to stop,
	# so it is refused here rather than described. Unreachable from the one
	# caller that keeps the order, which is what makes saying so worth the
	# lines: the next caller of this is where it stops being unreachable.
	[ -n "$pod_source_line" ] ||
		die "this build staged an audio-device binary without checking where it came from, which is a bug in this script." \
			"refuse_unless_pod_checkout_matches is what fills that answer in, and it has to" \
			"run before the pod is built or staged."
	echo "$pod_source_line"
}

# What the stamp records for the pod half of the payload.
#
#   pod_stamp_field
#
# `<commit>[+dirty]` for a pod this build compiled, and `named` for one an
# operator handed it. The commit is the checkout's, which the refusals above have
# already held equal to what the container built and to what the voice host
# links, so unlike the `brenn_pod=` field beside it this one does describe the
# binary in the payload.
pod_stamp_field() {
	if [ -n "$pod_binary_named" ]; then
		echo named
		return
	fi
	if [ -z "$pod_commit" ]; then
		echo unknown
		return
	fi
	if [ "$pod_dirty" = true ]; then
		echo "${pod_commit}+dirty"
	else
		echo "$pod_commit"
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
#
# The third spelling, `none`, carries no speech configuration whatever the tree
# holds: the path is then empty, which every reader takes as "no file", and it
# is not named, so its absence is no refusal.
#
# `speech_config_named` says whether an operator named a file, which is the
# whole difference between "not there and that is the shipped state" and "not
# there and you asked for it".
case ${REACHY_SPEECH_CONFIG:-} in
none)
	# Carry no speech configuration whatever the tree holds — an operator's
	# choice for a payload that will not speak; the members staged beside a
	# speech configuration stay out with it.
	speech_config=
	speech_config_named=
	;;
'')
	speech_config=${repo_root}/host/speech.toml
	speech_config_named=
	;;
*)
	speech_config=$REACHY_SPEECH_CONFIG
	speech_config_named=named
	;;
esac

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

# One quoted-string scalar out of a protobuf-text file.
#
#   textproto_string <file> <field>
#
# One field per line, leading whitespace allowed. A missing file, an absent
# field and an empty value are three different refusals because they are three
# different edits.
textproto_string() {
	local file=$1 field=$2 line value
	[ -f "$file" ] ||
		die "there is no ${file}, so its ${field} cannot be read."
	line=$(sed -n "s/^[[:space:]]*\\(${field}: \".*\"\\)\$/\\1/p" -- "$file" | head -n 1)
	[ -n "$line" ] ||
		die "${file} states no ${field}, so the command that needs it cannot be built." \
			"If the field was renamed, this script and the runbook both name the old one."
	value=${line#*: \"}
	value=${value%\"}
	[ -n "$value" ] ||
		die "${file} states an empty ${field}, so the command that needs it would name nothing."
	printf '%s\n' "$value"
}

# The recording session's own speech configuration: a second arrangement of the
# voice pipeline (bypassed wake gate, echo brain, no bridge), and a site's file
# for the same reason `speech_config` is. The payload has one speech-config slot
# per launcher entry, so a second arrangement is a second file, staged beside
# the site's.
#
# Optional in the build, refused by `deploy-motion.sh --record`. Named and
# missing is a refused build. `none` carries none whatever the tree holds, as
# for the site's configuration.
#
# `record_speech_config_named` says whether an operator named a file, which is
# the difference between "not there and that is the shipped state" and "not
# there and you asked for it".
case ${REACHY_RECORD_SPEECH_CONFIG:-} in
none)
	record_speech_config=
	record_speech_config_named=
	;;
'')
	record_speech_config=${repo_root}/host/speech-record.toml
	record_speech_config_named=
	;;
*)
	record_speech_config=$REACHY_RECORD_SPEECH_CONFIG
	record_speech_config_named=named
	;;
esac

# Where it goes under the payload root: the path
# `host/host_record_launch.textproto` spells in its `--speech-config` argument.
# The two have to agree, and `tools/build-motion.test.sh` holds them to each
# other.
record_speech_config_path=host/speech-record.toml

# The bench's own configuration: the serial node this unit's servos are on. A
# per-unit file, never in this tree. One knob shared with the bench payload,
# because both name the same physical serial node.
#
# A relative value is relative to this repository's root: the Makefile's default
# is spelled relatively, so a value arriving here may be one no human typed.
#
# Optional in the build; `deploy-motion.sh --record` refuses a payload without
# one.
bench_config=${BENCH_CONFIG:-${repo_root}/.local/reachy-bench.toml}
case $bench_config in
/*) ;;
*) bench_config=${repo_root}/${bench_config} ;;
esac

# Where it goes under the payload root: the path
# `bench/record_launch.textproto` spells in the recorder's `--config` argument.
# `tools/build-motion.test.sh` holds the two to each other.
bench_config_path=bench/reachy-bench.toml

# ---------------------------------------------------------------------------
# The payload a run is stamped and packed from
# ---------------------------------------------------------------------------
#
# Here rather than in the push because two scripts write the stamp: the push
# into the payload it rsyncs, the pack into the payload it archives. One list,
# one digest, one function, so a fetched run's stamp and a pushed run's say the
# same things in the same words.

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

# The sha256 of one file, the digest alone.
sha256_of() {
	sha256sum -- "$1" | cut -d' ' -f1
}

# What the device payload is built out of: the sources, everything that decides
# how they are compiled, the compositions and the configuration the processes
# read, the two scripts that decide what a built payload is -- the one that
# names the platform and the compilation mode, and the shared prelude it takes
# its ELF verification from -- and the payload's own entry point, which the
# build stages as `run`.
workspace_paths=(
	crates cogs driver motion hardware geometry clips bazel
	MODULE.bazel MODULE.bazel.lock .bazelrc .bazelversion
	tools/build-motion.sh tools/lib.sh tools/payload-run.sh
)

# What build a run's records came off, into the file a run carries home.
#
#   stamp_provenance <file> <yes|no: age unchecked> <push|pack: who is writing> <instant, %Y%m%dT%H%M%SZ>
#
# The log reader binds each channel's schema byte for byte, so a run's records
# are read with the build that recorded them and a records directory that cannot
# name its build is one nobody can decode after the next `.clk` append. Nothing
# else in a fetch says which build it was.
#
# What it can honestly claim is narrow, and it claims exactly that. The push- or
# pack-time facts are weaker than they look: the freshness refusal compares the payload's
# age against the newest commit and does not catch uncommitted edits, and
# --stale-ok skips it altogether. And the stamping tree's HEAD is not by itself
# the commit the binaries came from: a payload built at one commit can be pushed
# from a checkout at any other, and the age refusal only turns away a payload
# that is too old — an older checkout passes it and would be stamped with a
# commit that never produced the binaries. So the commit the stamp names is the
# one the build recorded in the payload (`build_commit_name`, lib.sh) whenever
# the payload carries it, `commit_source` says which of the two answered, and
# `stamped_from` keeps the stamping tree's HEAD beside it so a tree that moved
# between the build and the stamp is visible rather than averaged away.
#
# The rest is the same honesty: whether the tree had uncommitted changes when it
# was stamped, and whether the age was checked at all — a dirty or stale stamp says
# so on its face instead of lying by omission, and a clean fresh one makes
# reading the log a `git switch --detach`.
#
# A tree that cannot state its commit is a push or pack refusal, not a stamp saying
# nothing: the whole point of the file is that a fetched log names its build.
#
# Beside the build it names the configuration: a `config_sha256=` line per file a
# run can be varied by, and the overlay directory that produced them, if any. The
# files themselves travel home in the log root's `config/` and are what the
# analyzers read; these lines are the push's or the pack's own record of what it
# staged, so a fetched log says both what its configuration is and that nothing
# rewrote it between the push or pack and the run.
#
# And a `wake_model_sha256=` line per site-supplied model the payload carries,
# out of `staged_wake_models` — the listing the caller already read from the staged
# speech configuration. The commit does not name that file: it is the site's own,
# not a fetch this tree pins, and a retrained head arrives in the assembly
# directory under the name of the one before it. So the digest is the only thing
# a fetched run can be attributed to a head by, which is what reading scores
# across sessions and across heads needs.
#
# Reads the caller's `payload`, `experiment_dir` and `staged_wake_models`.
stamp_provenance() {
	local into=$1 age_unchecked=$2 stamped_by=$3 instant=$4
	case $stamped_by in
	push | pack) ;;
	*) die "stamp_provenance: stamped_by must be push or pack, not '${stamped_by}'" ;;
	esac
	local stamped_from dirty built commit commit_source brenn_pod reachy_pod name
	stamped_from=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null) || stamped_from=
	[ -n "$stamped_from" ] ||
		die "this tree cannot state its own commit, so a push or pack from it could not say which build ran." \
			"Every fetched records directory carries that commit, because a log is only" \
			"readable by the build that recorded it. Run it from a checkout with history."
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
		commit=$stamped_from
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
# Which build recorded the records beside this file. Written by ${prog} into the
# payload and copied here by the run.
#
# The log reader binds a channel's schema byte for byte, so read these records
# with the build that wrote them:
#     git switch --detach ${commit}
#
# commit_source=build means the payload itself recorded that commit when it was
# staged, which is the build the binaries came out of. commit_source=push means
# the payload recorded none and this is the stamping tree's HEAD instead, which
# describes the binaries only if that tree had not moved since the build. A pack
# refuses a payload that recorded no commit, so a pack's stamp is always
# commit_source=build. stamped_from is the stamping tree's HEAD either way: where
# it differs from commit, the tree moved between the build and the stamp and
# commit is the one that built.
#
# stamped_by says which script wrote this stamp: push, into the payload
# deploy-motion.sh rsynced to a unit; pack, into the payload pack-motion.sh
# archived for the boot fetch, a resync or a bake. stamped is the instant of
# that write for a push. For a pack it is the build commit's instant instead,
# so an archive's bytes -- and the digest a bake records of it -- depend on the
# tree alone; it says nothing about when the archive was packed, fetched or run.
#
# dirty=yes means the workspace held uncommitted changes when the stamp was
# written, so that commit does not fully describe what ran. dirty=unknown means
# the repository would not answer the status question then, so whether there
# were any is not known. age_unchecked=yes means the push or pack skipped the
# refusal that compares the payload's age against the newest commit, so the
# payload may predate that commit.
#
# overlay names the directory of experiment configuration the push laid over the
# payload, or none (a pack lays none). A config_sha256 line per file a run can be
# varied by, over the copy that was stamped: the same files are in config/
# beside these records, so a digest that disagrees with one of them is a payload
# edited on the unit.
#
# A wake_model_sha256 line per model the speech configuration supplies itself,
# naming its payload path and the digest of the copy that was stamped. The wake
# head is a site file rather than a fetch this tree pins, and heads are retrained
# under one name, so the commit above says nothing about which one this run
# listened with. No such line means the payload carried no site-supplied model.
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
stamped_by=${stamped_by}
stamped_from=${stamped_from}
stamped=${instant}
dirty=${dirty}
age_unchecked=${age_unchecked}
overlay=${experiment_dir:-none}
STAMP
	for name in "${run_config_files[@]}"; do
		echo "config_sha256=${name} $(sha256_of "${payload}/${name}")" >>"$into"
	done
	local model_path
	while IFS=$'\t' read -r _ model_path _; do
		[ -n "$model_path" ] || continue
		echo "wake_model_sha256=${model_path} $(sha256_of "${payload}/${model_path}")" >>"$into"
	done <<<"$staged_wake_models"
	echo "${prog}: provenance: commit ${commit} (${commit_source}), stamped by ${stamped_by} from ${stamped_from}," \
		"dirty=${dirty}, age_unchecked=${age_unchecked}" >&2
}

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

# Refuse a payload-relative path the payload cannot carry.
#
#   check_payload_path <config> <key> <value> <noun>
#
# Every file the configuration names travels inside the payload and is named by
# the payload-relative path it will occupy — which is also the path the host
# resolves at run time, because the launcher starts it with the payload root as
# its working directory. An absolute path is the workstation-era spelling: it would resolve on
# the machine the payload was built on and name nothing on the unit.
#
# The noun is what the file is to an operator ("credential", "wake model"): the
# two refusals that name the kind of file take it, because these rules are about
# payload-relative paths and not about any one class of member. It is used in the
# singular in both, so a caller's noun is never bent into a plural this cannot
# spell.
check_payload_path() {
	local config=$1 key=$2 value=$3 noun=$4
	case "$value" in
	/*)
		die "${key} in ${config} is the absolute path ${value}, and the payload carries its own copy of every ${noun}." \
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
			"that resolves to one of them without matching it would install a ${noun}" \
			"over a launcher config or a model. Name the file plainly."
		;;
	esac
}

# The site files a speech configuration names, as
# `<key>\t<payload-relative path>\t<source path>` lines, one per key it states.
#
#   payload_path_entries <config> <noun> <table\tkey>...
#
# One reader for every class of file that travels by the assembly-directory
# convention, because the three columns and the rules behind them are one
# contract: a second copy of this loop is a second place a new column or a new
# refusal has to be made, with separate suites that both go green when only one
# of them is.
#
# A configuration that is not there names nothing: a payload built without one
# is the ordinary case, and the build refuses a *named* one that is missing
# before this is ever asked. Every value that is there is checked here, so the
# build and the push cannot disagree about which paths the payload's site files
# occupy.
#
# The source column is the other half of the convention: the configuration names
# the path a file will occupy in the payload, and the file itself sits beside the
# configuration under that same name. Emitted here rather than re-joined at each
# call site, so the build's staging and the push's freshness check cannot come to
# disagree about where a file came from — which would be a stale secret, or a
# superseded wake head, shipped under a green verdict.
#
# Read through a command substitution, as `toml_table_value` is.
payload_path_entries() {
	local config=$1 noun=$2
	shift 2
	local entry table key value
	[ -f "$config" ] || return 0
	for entry in "$@"; do
		table=${entry%%$'\t'*}
		key=${entry#*$'\t'}
		value=$(toml_table_value "$config" "$table" "$key") || exit 1
		[ -n "$value" ] || continue
		check_payload_path "$config" "$key" "$value" "$noun"
		printf '%s\t%s\t%s\n' "$key" "$value" "$(dirname -- "$config")/${value}"
	done
}

# The credential files a speech configuration names.
#
#   speech_credential_paths <config>
speech_credential_paths() {
	payload_path_entries "$1" credential "${speech_credential_keys[@]}"
}

# The speech configuration's model path fields, as `<table>\t<key>`.
#
# One key, and a list rather than a bare name because the reader takes a list and
# because a second site-supplied model would be a row here. `[wake]
# melspectrogram`, `[wake] embedding` and `[endpointer] model`
# are deliberately not in it: those name files the build fetches and stages from
# its own table, so the scripts never read them and the host's `--check`
# preflight is what holds them to the payload.
speech_model_keys=(
	$'wake\tmodel'
)

# The model files a speech configuration supplies itself.
#
#   speech_model_paths <config>
#
# The wake head is the one model that is site policy rather than build input:
# which phrase this machine answers to. So it travels by the assembly-directory
# convention the credentials travel by — the configuration names the
# payload-relative path the file will occupy, and the file sits beside the
# configuration under that same name — rather than being fetched by digest with
# the phrase-independent graphs.
#
# Absent keys name nothing, as for credentials: a configuration with no `[wake]`
# table, or one whose wake mode reads no head, stages none. Whether what is left
# is a coherent pipeline is the host's business and surfaces at `--check`.
speech_model_paths() {
	payload_path_entries "$1" "wake model" "${speech_model_keys[@]}"
}

# The speech configuration's clip path fields, as `<table>\t<key>`. One key: the
# line the host speaks when a wake's transcription fails. Its own list and noun,
# not the wake model's, so a refusal about it names the right file.
speech_clip_keys=(
	$'stt\tunreachable_clip'
)

# The clips a speech configuration supplies itself.
#
#   speech_clip_paths <config>
speech_clip_paths() {
	payload_path_entries "$1" "offline clip" "${speech_clip_keys[@]}"
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
# Echoes the newest directory directly under the log root whose name is the
# logger's stamp -- decimal digits only, the instant it opened the log in
# nanoseconds, as in `1788832560362471129` -- and that holds a non-empty
# `.olog`. Other names are skipped: the log root also carries the `config/`
# copy a deploy leaves beside the runs, and a sort that took the newest of
# everything would hand a caller a directory holding no records at all. Newest
# is a numeric sort, because a lexicographic one only agrees with it while
# every stamp in a root is the same width. Both failures are
# refusals rather than a report over nothing: an empty log root is the
# namespace-mismatch failure that survives a whole session unnoticed if
# anything downstream is willing to read zero records. The two hints are the
# caller's, because where a process's console output landed and which
# configuration file states the namespace differ between a host staging tree
# and a device fetch.
run_directory() {
	local logs=$1 no_directory=$2 no_records=$3
	local dir found
	# `|| :` on the filter: no stamp-named directory is an empty selection and
	# the refusal below, never a pipeline status. Under `set -o pipefail` a
	# `grep` that matched nothing exits the caller at this assignment with no
	# message, and the two hints are the whole reason this function exists.
	dir=$(find "$logs" -mindepth 1 -maxdepth 1 -type d -print \
		| { grep -E '/[0-9]+$' || :; } \
		| awk -F/ '{ print $NF "\t" $0 }' | sort -n | tail -n 1 | cut -f2-)
	[ -n "$dir" ] || die \
		"the logger wrote no stamp-named run directory under ${logs}." \
		"$no_directory"
	found=$(find "$dir" -name '*.olog' -size +0 -print -quit)
	[ -n "$found" ] || die \
		"${dir} holds no non-empty .olog file." "$no_records"
	echo "$dir"
}

# Run one of this tree's log analyzers, and let its verdict be the caller's.
#
#   analyzer_verdict <analyzer target> [analyzer arguments...]
#
# One invariant behind all three analyzers, stated once here rather than beside
# each of them. An analyzer is a host tool over a log that has stopped being
# written, so it builds in the default configuration whatever configuration the
# payload was built in: callers pass $bazel and $build_flags, and the device
# harness's $build_flags is deliberately empty for that reason. The arguments
# and their order are the analyzer's own, and the exit status is returned rather
# than judged, because it is the analyzer's verdict and not this wrapper's.
analyzer_verdict() {
	local target=$1
	shift
	"$bazel" run "${build_flags[@]}" -- "$target" "$@"
}

# The label of the analyzer that judges a run's records. One string, because
# both run harnesses invoke the same analyzer over their own fetched or staged
# log and a rename has to reach both.
report_target=//cogs:first_motion_report

# Judge a run's records.
#
#   report_verdict <run directory> [extra analyzer arguments...]
#
# The extra arguments are the caller's — the host run reads a staged log with a
# jitter band, a device run reads hardware timestamps strictly — and they go
# ahead of the run directory, which is this analyzer's grammar.
#
# The configuration the run was performed under is no argument: the analyzer
# reads it out of the run directory's own `config/`, which is where every path
# that produces a log puts it. A profile named here instead would be the
# analyzing host's answer to a question only the machine that ran can answer.
report_verdict() {
	local run_dir=$1
	shift
	analyzer_verdict "$report_target" "$@" "$run_dir"
}

# The analyzer of a speech run, which reads both sides of a fetch: what a
# supervised session holds depends on what a person said to the robot, so the
# pipeline's own narration is the evidence for what was asked and the records
# are the evidence for what the head did with it. The motion analyzer's grid
# arithmetic still has nothing to say about a session nobody budgeted.
speech_report_target=//cogs:speech_run_report

# Judge a speech run's fetch.
#
#   speech_verdict <fetched records directory>
#
# The argument is the fetch's own directory, not a run directory inside it: the
# analyzer names the console beside it from that spelling and finds the run
# directory within it the way `run_directory` above does.
speech_verdict() {
	analyzer_verdict "$speech_report_target" "$1"
}

# The analyzer of a recording session, which reads a fetch holding no records at
# all: the composition that recorded it runs no logger, and the pose stream and
# the transcripts are both consoles. Its document is what an operator and an LLM
# turn into clips.
pose_report_target=//cogs:pose_session_report

# Judge a recording session's fetch, and write its document.
#
#   pose_verdict <fetched records directory>
#
# The document goes beside the fetch under the fetch's own name, the way the
# console and the audio store do: a session's segment ids are only meaningful
# under the configuration that produced them, so the document that names them
# belongs to that fetch and not to a shared output directory.
pose_verdict() {
	analyzer_verdict "$pose_report_target" "$1" --out "${1}.session"
}

# The analyzer of a library tour, which judges a run against the library it was
# supposed to play: the motion analyzer reads one gesture off the records and
# has no list of what should have happened, and a tour's whole question is
# whether every motion in the sidecar was asked for and moved the machine.
tour_report_target=//cogs:library_tour_report

# Judge a library tour's records.
#
#   tour_verdict <run directory> <names.json>
#
# The run directory is the one `run_directory` found, and the sidecar is the
# committed name table the tour was built from -- both absolute, because this
# runs under `bazel run` from its own runfiles tree. The configuration is read
# out of the run directory, for the reason `report_verdict` gives.
tour_verdict() {
	analyzer_verdict "$tour_report_target" "$1" "$2"
}

script_report_target=//cogs:script_run_report

script_verdict() {
	local run_dir=$1 names=$2
	shift 2
	analyzer_verdict "$script_report_target" "$run_dir" "$names" "$@"
}
