#!/usr/bin/env bash
#
# Put the built bench binary on a device and run it there.
#
#   tools/deploy-bench.sh <host> --config <file>
#   tools/deploy-bench.sh <host> --run [args...]
#   tools/deploy-bench.sh <host> --fetch <dir>
#
#   --config  push the bench's configuration into the account's home on the
#             device. Separate from --run because the configuration changes
#             rarely and a bench session is many runs.
#   --run     run the binary out of the pushed release, as the account the
#             payload runs as, with everything after --run passed to it
#             verbatim. Pushes the binary first.
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

usage() {
	die "usage: ${prog} <host> --config <file>|--run [args...]|--fetch <dir>"
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
