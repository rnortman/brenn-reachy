#!/usr/bin/env bash
#
# Provision the pod's half of the voice link on a unit, through brenn-pod.
#
#   tools/provision-speech.sh <host>
#
# The audio device dials the voice host over a PSK-authenticated loopback
# connection, and its end of that — the address and the key, at
# /run/brenn-app/conf/audio.conf — is written by brenn-pod's
# `reachy-provision`, from the same speech configuration this repo's build
# stages into the payload. brenn-reachy never writes that file and never
# reimplements the derivation: one writer, in the repo that owns the format.
# This script only invokes that writer, through its supported make target, with
# the arguments a speech run implies.
#
# It is a step of every speech run rather than a thing an operator remembers.
# `audio.conf` is on tmpfs, so a rebooted unit has lost it, and the command that
# restores it is idempotent, so running it unconditionally makes a cold unit and
# a warm one the same prompt.
#
# `ON_UNIT=1` is not passed by the operator either: it names the *arrangement*
# being provisioned — the voice host runs on the unit, so a loopback
# `listen_addr` and a config-relative `pod_psk_file` are the right shapes rather
# than refusable ones — and a speech run is that arrangement by definition.
# Nothing here runs on the device: brenn-pod's script is a workstation program
# whose only device contact is ssh.
#
# Knobs, environment only:
#
#   REACHY_SPEECH_CONFIG   the assembly configuration (default: host/speech.toml)
#   BRENN_POD_DIR          the brenn-pod checkout (default: ../brenn-pod)

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

[ "$#" -eq 1 ] || die "usage: ${prog} <host>"
host=$1
# An empty argument is an unset variable that expanded inside quotes, not a host.
# Refused here so the error speaks in this tool's vocabulary.
[ -n "$host" ] || die "usage: ${prog} <host>"

pod_firmware="${brenn_pod_dir}/firmware"

# Both refusals are asked here, in this repo's vocabulary, rather than left to
# the other repo's script, whose messages speak to its own operators rather than
# ours.
mapfile -t config_remedy < <(knob_remedy REACHY_SPEECH_CONFIG "<assembly>/speech.toml")
[ -f "$speech_config" ] ||
	die "there is no speech configuration at ${speech_config}, so the pod's half of the link cannot be derived." \
		"That file is the assembly directory's, and it is what both halves of the link come from." \
		"${config_remedy[@]}"

mapfile -t checkout_remedy < <(knob_remedy BRENN_POD_DIR "<path to brenn-pod>")
[ -f "${pod_firmware}/Makefile" ] ||
	die "there is no brenn-pod checkout at ${brenn_pod_dir}, which is where the pod's provisioning lives." \
		"The default is a sibling of this checkout." \
		"${checkout_remedy[@]}"

# Absolute, because a relative SPEECH_CONFIG is resolved against brenn-pod's own
# repository root and this one's default is relative to ours.
speech_config_abs=$(cd -- "$(dirname -- "$speech_config")" && pwd)/$(basename -- "$speech_config")

# Both values on the sub-make's command line, where nothing overrides them: a
# brenn-pod checkout carries a `.local/reachy.conf` of its own, and a file-origin
# value there would otherwise decide which unit or which configuration this run
# provisioned — the two repos disagreeing about a link they are two halves of.
#
# The parent make's flags are dropped: brenn-pod's build is not a part of this
# one's dependency graph, and inheriting a jobserver it cannot join costs a
# warning and buys nothing.
echo "${prog}: provisioning ${host}'s audio device from ${speech_config_abs}" >&2
env -u MAKEFLAGS -u MAKELEVEL -u MFLAGS make -C "$pod_firmware" reachy-provision \
	ON_UNIT=1 \
	SPEECH_CONFIG="$speech_config_abs" \
	REACHY_HOST="$host"
