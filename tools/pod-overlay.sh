#!/usr/bin/env bash
#
# tools/pod-overlay.sh — switch this tree's brenn-pod `crate.spec`s between the
# published pin and a working tree beside this checkout.
#
# The two forms are the ones the comment block above `BRENN_POD_REV` in
# `MODULE.bazel` describes: `git = BRENN_POD_GIT, rev = BRENN_POD_REV` resolves
# the surface's crates from the remote, and `path = "../brenn-pod/<crate path>"`
# resolves them from the tree next door. A cycle that lands a seam on both sides
# of the dependency arrow runs on the second one — the payload's pod is compiled
# from that same checkout, and `refuse_unless_pod_checkout_matches` refuses a
# payload whose two halves of brenn-pod disagree.
#
# Why a script and not four hand edits: `crate.spec` has no override form, so
# every spec is edited by hand and all four have to be in the same form at once.
# A half-finished edit is a voice host that links two revisions of brenn-pod,
# which is exactly what the build then refuses — after a cross-build. Doing it
# from one program makes the state a command instead of a careful moment, and
# makes `off` as cheap as `on`, which is what keeps a push from carrying one.
#
#   tools/pod-overlay.sh on       # resolve the four specs from BRENN_POD_DIR
#   tools/pod-overlay.sh off      # put them back on BRENN_POD_GIT/BRENN_POD_REV
#   tools/pod-overlay.sh status   # which form the file is in, and from where
#
# `on` and `off` are idempotent: a file already in the asked-for form is left
# alone and says so.
#
# What this does not do, deliberately:
#
# - It does not touch `MODULE.bazel.lock`. Bazel rewrites it under a `path =`
#   spec with an absolute path in it, so the lock is left uncommitted for as
#   long as the overlay runs and is restored from git with the pin; `off` says
#   so rather than guessing which of its changes were the overlay's.
# - It does not push, commit or check anything. `make check-pins` at the push
#   and CI are the gate that an overlay never reaches a published ref.
#
# It does clear the overlay's litter on `off`: a `path =` spec makes bazel write
# a generated `BUILD.bazel` into each overlaid crate directory of the other
# repository, and an untracked one there is this tree's mess to clean up.

set -euo pipefail

prog=$(basename -- "$0")
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=lib.sh
. "${script_dir}/lib.sh"

module="${MODULE_OVERLAY_FILE:-${repo_root}/MODULE.bazel}"

# Where each brenn-pod package's crate lives inside that checkout. Stated here
# rather than searched for: a package whose directory moved should stop this
# script with a name, not be overlaid from a path that resolves to something
# else.
crate_dir_for() {
	case $1 in
	speech-surface) echo host/crates/speech-surface ;;
	speech-pipeline) echo host/crates/speech-pipeline ;;
	pod-ingest) echo host/crates/pod-ingest ;;
	brenn-bridge) echo firmware/crates/brenn-bridge ;;
	*) return 1 ;;
	esac
}

# The same map as one `pkg=dir;` string for the rewriter's awk, so the mapping
# is stated once: a package `tools/lib.sh` names and this script has no
# directory for stops here, by name, before anything is written.
crate_dir_map() {
	local pkg dir map=""
	for pkg in $pod_crate_packages; do
		dir=$(crate_dir_for "$pkg") ||
			die "${prog} has no crate directory for the brenn-pod package ${pkg}." \
				"tools/lib.sh names the packages and this script names where each one" \
				"lives; a package added to one is added to both."
		map="${map}${pkg}=${dir};"
	done
	printf '%s\n' "$map"
}

# How many brenn-pod specs a rewrite has to touch: the package list's own
# length, so a fifth package is a fifth spec and not a refusal nobody can place.
pod_package_count() {
	local pkgs
	read -r -a pkgs <<<"$pod_crate_packages"
	printf '%s\n' "${#pkgs[@]}"
}

usage() {
	die "usage: ${prog} on|off|status"
}

# The overlaid tree as the file spells it: relative to this repository's root,
# because that is what `pod_overlay_paths`'s readers resolve it against and what
# a second machine with the same two checkouts side by side can load.
overlay_base() {
	local rel
	rel=$(realpath --relative-to="$repo_root" -- "$brenn_pod_dir" 2>/dev/null) ||
		die "${brenn_pod_dir} cannot be expressed relative to ${repo_root}, so no crate.spec can name it." \
			"A path spec is resolved against this repository's root. Put the two" \
			"checkouts side by side, or name the overlay by hand."
	printf '%s\n' "$rel"
}

# Every package's crate directory, checked before anything is written: half the
# specs overlaid onto directories that exist and half onto ones that do not is
# the mixed form arriving through a typo.
check_crates() {
	local base=$1 pkg dir
	for pkg in $pod_crate_packages; do
		dir=$(crate_dir_for "$pkg") ||
			die "${prog} has no crate directory for the brenn-pod package ${pkg}." \
				"tools/lib.sh names the packages and this script names where each one" \
				"lives; a package added to one is added to both."
		[ -d "${brenn_pod_dir}/${dir}" ] ||
			die "${brenn_pod_dir}/${dir} is not there, so the overlay would name a crate that does not exist." \
				"BRENN_POD_DIR names the checkout the specs resolve from; it stands at" \
				"${brenn_pod_dir}."
		[ -f "${repo_root}/${base}/${dir}/Cargo.toml" ] ||
			die "${base}/${dir}/Cargo.toml is not there, so ${pkg} would resolve to a directory that is no crate." \
				"The overlay is written relative to this repository's root."
	done
}

# Rewrite every brenn-pod spec into one form or the other, on stdout.
#
#   write_specs <MODULE.bazel path> <want: overlay|pinned> [relative base]
#
# One program for both directions: the two differ only in what replaces a
# matched spec's source attributes, and a second copy of the block grammar is a
# second reader of `MODULE.bazel` to keep in step with `pod_overlay_paths`.
#
# A line reader over the literals a person typed, in the same shape and with the
# same preconditions as `pod_overlay_paths`: `crate.spec(` alone on its line, one
# attribute per line, comments skipped. A spec written any other way is left
# exactly as it is and counted, and the caller refuses on the count — a silent
# pass over a spec this cannot read is how a mixed file is written.
#
# Statuses: 2 for a spec it cannot read, 3 when it did not rewrite every
# package, 4 for a package it has no crate directory for.
write_specs() {
	awk -v packages="$pod_crate_packages" -v want="$2" -v base="${3-}" \
		-v dirs="$(crate_dir_map)" -v want_count="$(pod_package_count)" -- '
		function flush(   i) {
			if (!index(packages, " " pkg " ")) {
				for (i = 1; i <= n; i++) print block[i]
				n = 0
				pkg = ""
				return
			}
			for (i = 1; i <= n; i++) {
				if (block[i] ~ /^[ \t]*git[ \t]*=[ \t]*BRENN_POD_GIT,/) continue
				if (block[i] ~ /^[ \t]*rev[ \t]*=[ \t]*BRENN_POD_REV,/) continue
				if (block[i] ~ /^[ \t]*path[ \t]*=[ \t]*"/) continue
				if (block[i] !~ /^[ \t]*package[ \t]*=[ \t]*"/) {
					print block[i]
					continue
				}
				# Attributes are in alphabetical order: `git` sorts
				# before `package`, `path` and `rev` after it.
				if (want == "pinned") {
					print "    git = BRENN_POD_GIT,"
					print block[i]
					print "    rev = BRENN_POD_REV,"
				} else {
					if (!(pkg in dir)) {
						unmapped = pkg
						exit 4
					}
					print block[i]
					printf("    path = \"%s/%s\",\n", base, dir[pkg])
				}
				wrote++
			}
			n = 0
			pkg = ""
		}
		BEGIN {
			pairs = split(dirs, entry, ";")
			for (i = 1; i <= pairs; i++) {
				if (entry[i] == "") continue
				at = index(entry[i], "=")
				dir[substr(entry[i], 1, at - 1)] = substr(entry[i], at + 1)
			}
		}
		/^[ \t]*#/ { if (inspec) block[++n] = $0; else print; next }
		# Defence behind the entry guard: every subcommand reads the form with
		# pod_overlay_paths first, which already refuses a compact spec, so this
		# is what keeps the rewriter honest if that guard is ever narrowed.
		/crate\.spec\(/ && !/^crate\.spec\($/ { unreadable++; print; next }
		/^crate\.spec\($/ { inspec = 1; n = 0; pkg = ""; block[++n] = $0; next }
		inspec && /^\)/ {
			block[++n] = $0
			flush()
			inspec = 0
			next
		}
		inspec {
			block[++n] = $0
			if ($0 ~ /^[ \t]*package[ \t]*=[ \t]*"/) {
				pkg = $0
				sub(/^[^"]*"/, "", pkg)
				sub(/".*/, "", pkg)
			}
			next
		}
		{ print }
		END {
			if (unmapped != "") exit 4
			if (unreadable) exit 2
			if (wrote != want_count) exit 3
		}
	' "$1"
}

# What a failed rewrite means, by the status the writer exited with. The
# temporary is gone and the target still holds every byte it started with.
refuse_rewrite() {
	local status=$1 tmp=$2
	rm -f -- "$tmp"
	case $status in
	4)
		die "${prog} has no crate directory for one of the packages in ${module}, so it was left alone." \
			"tools/lib.sh names the packages and this script names where each one" \
			"lives; a package added to one is added to both."
		;;
	*)
		die "${module}'s brenn-pod specs were not all rewritten, so it was left alone." \
			"Either a spec is written in a form this cannot read — the form is" \
			"crate.spec( alone on its line, one attribute per line — or the file does" \
			"not carry all $(pod_package_count) packages."
		;;
	esac
}

# Write `new` over `module`, once every spec was rewritten and the result reads
# back as the form that was asked for.
#
#   install <new file> <want: overlay|pinned>
#
# Through a temporary file next to the target and a rename: a MODULE.bazel
# truncated by a failed write is a tree that loads nothing at all. The form is
# read back off the temporary and the rename only follows a result that is the
# one asked for, so every refusal in this script leaves the target byte-identical.
install_module() {
	local new=$1 want=$2 source
	source=$(pod_build_source "$new")
	if [ "$source" = unknown ]; then
		rm -f -- "$new"
		die "the rewrite of ${module} does not read as one form or the other, so it was left alone." \
			"The rewritten specs are half overlaid and half pinned, or written in a" \
			"form tools/lib.sh cannot read. Read the file before building anything."
	fi
	case $source in
	overlay:*)
		if [ "$want" != overlay ]; then
			rm -f -- "$new"
			die "the rewrite of ${module} still reads as an overlay, so it was left alone." \
				"Nothing was meant to be left overlaid; read the file."
		fi
		;;
	*)
		if [ "$want" != pinned ]; then
			rm -f -- "$new"
			die "the rewrite of ${module} still reads as the pin, so it was left alone." \
				"No spec was rewritten; read the file."
		fi
		;;
	esac
	mv -- "$new" "$module"
}

# The overlay's generated litter in the other repository: one `BUILD.bazel` per
# overlaid crate directory, written by bazel and tracked by nobody. Removed only
# when git there says it is untracked, so a repository that has taken to
# tracking its own is left alone.
clear_litter() {
	local pkg dir file
	git -C "$brenn_pod_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1 || return 0
	for pkg in $pod_crate_packages; do
		dir=$(crate_dir_for "$pkg") || continue
		file="${brenn_pod_dir}/${dir}/BUILD.bazel"
		[ -f "$file" ] || continue
		if git -C "$brenn_pod_dir" ls-files --error-unmatch -- "${dir}/BUILD.bazel" >/dev/null 2>&1; then
			continue
		fi
		rm -f -- "$file"
		echo "${prog}: removed the overlay's generated ${dir}/BUILD.bazel"
	done
}

[ $# -eq 1 ] || usage
[ -f "$module" ] || die "${module} is not there, so there are no specs to switch."

case $1 in
status)
	source=$(pod_build_source "$module")
	case $source in
	overlay:*)
		echo "${prog}: overlaid from ${source#overlay:}"
		echo "${prog}: the pod binary is built from ${brenn_pod_dir}"
		;;
	unknown)
		die "${module}'s brenn-pod specs read as neither form." \
			"Half overlaid and half pinned, or written in a form tools/lib.sh cannot" \
			"read. Read the file."
		;;
	*) echo "${prog}: pinned at ${source:0:12}" ;;
	esac
	;;
on)
	source=$(pod_build_source "$module")
	case $source in
	overlay:*)
		echo "${prog}: already overlaid from ${source#overlay:}"
		exit 0
		;;
	unknown)
		die "${module}'s brenn-pod specs read as neither form, so there is nothing to switch from." \
			"Half overlaid and half pinned, or written in a form tools/lib.sh cannot" \
			"read. Read the file."
		;;
	esac
	base=$(overlay_base)
	check_crates "$base"
	tmp="${module}.overlay.$$"
	status=0
	write_specs "$module" overlay "$base" >"$tmp" || status=$?
	[ "$status" -eq 0 ] || refuse_rewrite "$status" "$tmp"
	install_module "$tmp" overlay
	echo "${prog}: every brenn-pod spec now resolves from ${base}"
	echo "${prog}: MODULE.bazel.lock records an absolute path under an overlay — leave it uncommitted"
	echo "${prog}: put the pin back before a push: tools/pod-overlay.sh off"
	;;
off)
	source=$(pod_build_source "$module")
	case $source in
	overlay:*) ;;
	unknown)
		die "${module}'s brenn-pod specs read as neither form, so there is nothing to switch from." \
			"Half overlaid and half pinned, or written in a form tools/lib.sh cannot" \
			"read. Read the file."
		;;
	*)
		echo "${prog}: already pinned at ${source:0:12}"
		exit 0
		;;
	esac
	rev=$(pinned_pod_rev "$module")
	[ -n "$rev" ] ||
		die "${module} states no BRENN_POD_REV this can read, so the specs have no revision to go back to." \
			"The pin is one line: BRENN_POD_REV = \"<40 hex digits>\"."
	tmp="${module}.pinned.$$"
	status=0
	write_specs "$module" pinned >"$tmp" || status=$?
	[ "$status" -eq 0 ] || refuse_rewrite "$status" "$tmp"
	install_module "$tmp" pinned
	clear_litter
	echo "${prog}: every brenn-pod spec is back on BRENN_POD_REV (${rev:0:12})"
	echo "${prog}: the lock was left as it is — restore the pinned one with"
	echo "${prog}:     git -C ${repo_root} checkout -- MODULE.bazel.lock"
	;;
*) usage ;;
esac
