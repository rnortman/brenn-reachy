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
#             accumulate rather than overwrite. Any hold-probe series sitting
#             beside it comes too, under the names the bench gave them. Either
#             half may be absent — only a selftest writes the state file, and
#             only a probe writes a series — and finding neither is the failure.
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

# What the bench names a hold-probe series. The binary states it as
# HOLD_PROBE_SERIES_PREFIX and this is the only other statement of it; the
# script's test compares the two, because nothing else joins a Rust format
# string to a shell glob and a rename on one side would leave this fetching
# nothing — which is the fetch's ordinary, silent case.
probe_prefix="hold-probe-"

# One directory, reused. Nothing on this path activates a release, so nothing
# prunes the store either; rsync --delete is what makes reuse idempotent.
release="${store_mount}/releases/bench"

# What the device binary is built out of: the sources, and everything that
# decides how they are compiled — the build files, the module and its lockfile,
# the flags, the Bazel release, and the two scripts that decide what a built
# binary is: the one that names the platform and the compilation mode, and the
# shared prelude it takes its ELF verification from. cogs/ is not an input to the
# bench and stays out.
workspace_paths=(
	crates bazel MODULE.bazel MODULE.bazel.lock .bazelrc .bazelversion
	tools/build-bench.sh tools/lib.sh
)

usage() {
	die "usage: ${prog} <host> --config <file>|--run [--stale-ok] [args...]|--fetch <dir>"
}

host=${1:-}
mode=${2:-}
[ -n "$host" ] || usage
shift 2 || usage

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
		# A missing state file is not the end of a fetch. Only `selftest`
		# writes one, ${app_home} is RAM, and the probe's own procedure is
		# runs of `hold-probe` and then a fetch — so dying here would
		# strand every series the session took behind an unrelated sweep.
		# Whether the fetch found anything at all is judged at the end.
		state_missing=""
		if ssh_root "cat ${app_home}/selftest-state.toml" >"$part"; then
			mv -- "$part" "$out"
			echo "${prog}: ${out}"
		else
			rm -f -- "$part"
			state_missing=1
			echo "${prog}: no state file on ${host}; only a selftest writes one." >&2
		fi

		# The probe's series, if any run wrote one. Each already carries
		# the moment it was taken and the servo it was taken from, so
		# they are fetched under their own names and a second fetch of
		# the same file is the same file. Absent is the ordinary case:
		# nothing has run the probe, and that is not a failed fetch.
		#
		# A tuning session is many runs and every one of them leaves a
		# series on the device, so the fetch is two connections whatever
		# the count: one listing, and one stream carrying every series
		# the device holds. Every one of them, not only the names that
		# are not here yet: the name carries a moment off a clock that
		# is RAM on a board with no battery, so two runs either side of
		# a boot without a network can wear the same name. What decides
		# "already here" is therefore the bytes, and a name here holding
		# different bytes is a collision an operator has to settle, not
		# something to overwrite or to skip.
		# TODO(bench-probe-series-retention): nothing removes a series
		# from the device, and ${app_home} is RAM.
		probes=$(ssh_root "ls -1 ${app_home}/${probe_prefix}*.csv 2>/dev/null" || true)
		wanted=()
		quoted=()
		for probe in $probes; do
			name=$(basename -- "$probe")
			# The bench's own shape, checked before the name goes
			# anywhere near a command line: the listing comes from a
			# directory the unprivileged account owns and this fetch
			# runs as root, so a name is untrusted input until it
			# has been read.
			[[ $name =~ ^${probe_prefix}[0-9]+-[0-9]+\.csv$ ]] || die \
				"unexpected file ${name} in ${app_home} on ${host}; the bench writes ${probe_prefix}<unix>-<id>.csv."
			wanted+=("$name")
			quoted+=("$(printf '%q' "$name")")
		done
		if [ ${#wanted[@]} -gt 0 ]; then
			# Into a staging directory first, for the reason the state
			# file uses a .part: a stream that fails partway must not
			# leave a truncated series under a name a later fetch then
			# takes for the whole one.
			part="${dest}/.probe-part"
			rm -rf -- "$part"
			mkdir -p -- "$part"
			ssh_root "tar -cf - -C ${app_home} -- ${quoted[*]}" | tar -xf - -C "$part" || {
				rm -rf -- "$part"
				die "could not fetch the probe series from ${host}."
			}
			# Every name checked before any of them is moved: a
			# stream that came back short must land nothing, or
			# the series that did arrive sit in the records
			# directory as though the fetch had finished.
			for name in "${wanted[@]}"; do
				[ -f "${part}/${name}" ] || {
					rm -rf -- "$part"
					die "${host} sent no ${name}."
				}
			done
			for name in "${wanted[@]}"; do
				if [ -f "${dest}/${name}" ]; then
					cmp -s -- "${part}/${name}" "${dest}/${name}" || {
						rm -rf -- "$part"
						die "${dest}/${name} is here already holding different readings; the device's clock reused a name. Move the local copy aside and fetch again."
					}
					echo "${prog}: ${dest}/${name} (already here)"
					continue
				fi
				mv -- "${part}/${name}" "${dest}/${name}"
				echo "${prog}: ${dest}/${name}"
			done
			rm -rf -- "$part"
		fi

		# Neither half was there: the fetch found nothing, which is a
		# failed fetch even though each half alone is allowed to be
		# absent.
		if [ -n "$state_missing" ] && [ ${#wanted[@]} -eq 0 ]; then
			die "nothing to fetch from ${host}: no state file and no probe series."
		fi
		;;

	--run)
		[ -x "$binary" ] || die \
			"no device binary at ${binary}" \
			"Build one first: make bench-build"

		if [ "${1:-}" = "--stale-ok" ]; then
			shift
			echo "${prog}: --stale-ok: the binary's age is not being checked" >&2
		else
			refuse_if_stale "$binary" "device binary" \
				"make bench-build" \
				"run the old binary deliberately" \
				"${prog} ${host} --run --stale-ok ..." \
				"${workspace_paths[@]}"
		fi

		echo "${prog}: pushing ${binary} to ${host}:${release}/" >&2
		ssh_root mkdir -p -- "$release"
		rsync -a --delete -e "ssh -o BatchMode=yes" \
			"$binary" "root@${host}:${release}/reachy-bench"

		# A bench run wants the servo bus to itself. Refused rather than
		# silently stopped: what is running on a device is the operator's to
		# decide.
		#
		# The question and the run are one remote invocation: asked
		# separately, the service can start in between and the run lands
		# beside it anyway. The question, its exit codes and the refusals
		# they turn into are lib.sh's, shared with the motion deploy —
		# they are one contract and the runbook documents them once.
		#
		# --init-groups is what puts the run in the dialout group, which is
		# what grants the serial node. Without it the drop would leave a run
		# with no supplementary groups at all and the port open would fail for
		# a reason that has nothing to do with the hardware.
		#
		# The working directory is the account's home, because that is where
		# the bench reads its configuration and writes its state.
		remote="$(bus_probe)"
		remote="${remote}; cd ${app_home} || exit 1"
		remote="${remote}; exec setpriv --reuid ${app_user} --regid ${app_user}"
		remote="${remote} --init-groups ${release}/reachy-bench"
		for arg in "$@"; do
			remote="${remote} $(printf '%q' "$arg")"
		done

		rc=0
		# A pty: the run prints as it goes and a ^C at the bench reaches it.
		ssh -t -o BatchMode=yes "root@${host}" "$remote" || rc=$?
		bus_refusal "$rc" "a bench run" "the bench did not run" \
			"Its own error is above. A run that never reached the board is not a hardware reading."
		# The bench's own verdict is this script's.
		exit "$rc"
		;;

	*) usage ;;
esac
