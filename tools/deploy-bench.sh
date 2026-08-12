#!/usr/bin/env bash
#
# Put the built bench binary on a device and run it there.
#
#   tools/deploy-bench.sh <host> --config <file>
#   tools/deploy-bench.sh <host> --run [--stale-ok] [args...]
#   tools/deploy-bench.sh <host> --fetch <dir>
#
#   --config  push the bench's configuration into the account's home on the
#             device. Separate from --run because the configuration changes
#             rarely and a bench session is many runs.
#   --run     run the binary out of the pushed release, as the account the
#             payload runs as, with everything after --run passed to it
#             verbatim. Pushes the binary first, and refuses a binary older
#             than the newest commit to the workspace.
#   --stale-ok  run the old binary anyway. This script's own flag, so it has to
#             be the first token after --run; everything from the next one on
#             belongs to the bench.
#   --fetch   copy the state file the bench writes back to a local directory,
#             named for the moment it was fetched so a session's runs
#             accumulate rather than overwrite.
#
# Two device paths, for two different reasons:
#
#   /run/brenn-app/releases  the binary. A tmpfs, so a push costs the device's
#       flash nothing and a reboot clears it. Binaries have to live here: /run
#       itself is mounted noexec and this submount deliberately is not.
#
#   /var/lib/brenn-app  the account's home — the configuration, and the state
#       file the bench writes. Also RAM. The release directory is root-owned
#       and rsync --delete owns its contents, so nothing the run produces can
#       live beside the binary.
#
# SSH lands as root and only root, which is why the run drops privilege: root
# opens any device node whatever udev said, so a serial port opened as root
# says nothing about the account that will hold it in normal operation.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

binary="${repo_root}/target/bench-arm64/release/reachy-bench"

# One directory, reused. Nothing on this path activates a release, so nothing
# prunes the store either; rsync --delete is what makes reuse idempotent.
release="${store_mount}/releases/bench"

service=brenn-app.service

# The other long-lived process on that unit, and the only other thing on it that
# opens the servo bus. It is deployed from brenn-pod and runs as a service, so a
# bench run at the wrong moment meets a held port rather than a free one.
motiond_service=reachy-motiond.service

# What the device binary is built out of. The container definition is in it:
# the builder image is named for that file's content, so editing it produces a
# different toolchain and a different binary.
workspace_paths=(crates containers Cargo.toml Cargo.lock rust-toolchain.toml)

usage() {
	die "usage: ${prog} <host> --config <file>|--run [--stale-ok] [args...]|--fetch <dir>"
}

# Refuse a device binary built before the newest commit that touched the
# workspace.
#
# The trap this closes cost a bench night: `bench-build` was taken for a
# once-per-session step, and a run whose findings looked like the machine's was
# reading a binary several commits old. Commit time against file time catches
# exactly that — it does not catch uncommitted edits, which is why `make
# bench-run` builds first and is the entry point to prefer.
#
# The rebuild this prescribes is what clears it, always: build-bench.sh stamps
# the artifact once it has verified it, because cargo leaves an output it did
# not have to relink untouched and a commit that changes no linked code would
# otherwise leave a current binary refused with no way through but --stale-ok.
#
# A tree with no history for those paths (a tarball, a shallow clone that
# excluded them) is not evidence of staleness, so it says what it could not
# decide and runs.
freshness() {
	local commit built
	commit=$(git -C "$repo_root" log -1 --format=%ct -- "${workspace_paths[@]}" 2>/dev/null) || commit=
	if [ -z "$commit" ]; then
		echo "${prog}: no commit history for the workspace here, so the binary's age is unknown" >&2
		return 0
	fi
	built=$(stat -c %Y -- "$binary") ||
		die "cannot read the age of ${binary}"
	[ "$built" -lt "$commit" ] || return 0
	die "the device binary is older than the newest commit to the workspace, so a run of it is not a run of this tree." \
		"Built $(date -d "@${built}" '+%Y-%m-%d %H:%M:%S'), newest commit $(date -d "@${commit}" '+%Y-%m-%d %H:%M:%S')." \
		"Rebuild it:" \
		"    make bench-build" \
		"or, to run the old binary deliberately:" \
		"    ${prog} ${host} --run --stale-ok ..."
}

host=${1:-}
mode=${2:-}
[ -n "$host" ] || usage
shift 2 || usage

ssh_root() {
	ssh -o BatchMode=yes "root@${host}" "$@"
}

case "$mode" in
	--config)
		config=${1:-}
		[ -n "$config" ] || usage
		[ -f "$config" ] || die "no configuration file at ${config}"

		# Over stdin rather than as an argument or a temporary file: the
		# contents never reach either machine's process table or shell
		# history. Mode 600 and owned by the account, because it is the
		# account that reads it.
		echo "${prog}: pushing ${config} to ${host}:${app_home}/reachy-bench.toml" >&2
		ssh_root "install -d -m 0700 -o ${app_user} -g ${app_user} -- ${app_home} &&
			install -m 0600 -o ${app_user} -g ${app_user} /dev/stdin ${app_home}/reachy-bench.toml" \
			<"$config"
		;;

	--fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		mkdir -p -- "$dest"
		stamp=$(date -u +%Y%m%dT%H%M%SZ)
		out="${dest}/selftest-state-${stamp}.toml"
		echo "${prog}: fetching ${host}:${app_home}/selftest-state.toml to ${out}" >&2
		# A redirection straight onto $out creates the file before the remote
		# command runs, so a fetch that fails leaves a zero-byte record
		# behind — and since every fetch is timestamped, nothing ever
		# overwrites it.
		part="${out}.part"
		ssh_root "cat ${app_home}/selftest-state.toml" >"$part" || {
			rm -f -- "$part"
			die "no state file on ${host}; the bench has not written one yet."
		}
		mv -- "$part" "$out"
		echo "${prog}: ${out}"
		;;

	--run)
		[ -x "$binary" ] || die \
			"no device binary at ${binary}" \
			"Build one first: make bench-build"

		if [ "${1:-}" = "--stale-ok" ]; then
			shift
			echo "${prog}: --stale-ok: the binary's age is not being checked" >&2
		else
			freshness
		fi

		echo "${prog}: pushing ${binary} to ${host}:${release}/" >&2
		ssh_root mkdir -p -- "$release"
		rsync -a --delete -e "ssh -o BatchMode=yes" \
			"$binary" "root@${host}:${release}/reachy-bench"

		# A bench run wants the servo bus to itself. Refused rather than
		# silently stopped: what is running on a device is the operator's to
		# decide.
		#
		# The refusal and the run are one remote invocation. Asked separately,
		# the service can start in between and the run lands beside it anyway;
		# and a check whose only signal is a nonzero exit reads ssh's own
		# failures (unreachable host, host-key refusal under BatchMode) as the
		# service being down. Exit 3 is this script's answer for "it is
		# running", and nothing else produces it.
		#
		# --init-groups is what puts the run in the dialout group, which is
		# what grants the serial node. Without it the drop would leave a run
		# with no supplementary groups at all and the port open would fail for
		# a reason that has nothing to do with the hardware.
		#
		# The working directory is the account's home, because that is where
		# the bench reads its configuration and writes its state.
		remote="systemctl is-active --quiet ${service} && exit 3"
		remote="${remote}; systemctl is-active --quiet ${motiond_service} && exit 4"
		remote="${remote}; cd ${app_home} || exit 1"
		remote="${remote}; exec setpriv --reuid ${app_user} --regid ${app_user}"
		remote="${remote} --init-groups ${release}/reachy-bench"
		for arg in "$@"; do
			remote="${remote} $(printf '%q' "$arg")"
		done

		rc=0
		# A pty: the run prints as it goes and a ^C at the bench reaches it.
		ssh -t -o BatchMode=yes "root@${host}" "$remote" || rc=$?
		case "$rc" in
			3)
				die "${service} is running on ${host}, and a bench run will not share the servo bus with it." \
					"Stop it, run the bench, and start it again when you are done:" \
					"    ssh root@${host} systemctl stop ${service}"
				;;
			4)
				die "${motiond_service} is running on ${host}, and it holds the servo bus." \
					"Stop it, run the bench, and start it again when you are done:" \
					"    ssh root@${host} systemctl stop ${motiond_service}" \
					"    ssh root@${host} systemctl start ${motiond_service}" \
					"(The port's own flock refuses either way; this is the message that says which process has it.)"
				;;
			255)
				die "ssh to root@${host} failed; the bench did not run." \
					"Its own error is above. A run that never reached the board is not a hardware reading."
				;;
		esac
		# The bench's own verdict is this script's.
		exit "$rc"
		;;

	*) usage ;;
esac
