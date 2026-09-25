#!/usr/bin/env bash
#
# Compose the audio device's link configuration for a payload, through
# brenn-pod's own writer.
#
#   tools/pod-conf.sh <pod-id> <speech-config> <out>
#
# The audio device dials the voice host over a PSK-authenticated loopback
# connection, and its end of that — the address and the key, a `KEY=VALUE`
# file — has one writer: brenn-pod's `firmware/tools/provision-reachy-pod.sh`,
# in the repo that owns the format. This script invokes it as
# `--on-unit --emit` and never reimplements the derivation.
#
# `--on-unit` names the arrangement: the voice host runs on the unit, so a
# loopback `listen_addr` and a config-relative `pod_psk_file` are the right
# shapes rather than refusable ones. `--emit` composes the file on stdout with no
# device contact at all.
#
# The pod id is the key table's name for the pod and the pod's TLS-PSK
# identity, which is the unit's hostname. When the table has no row for it the
# writer mints a key and files it beside the configuration, so the first build
# against a fresh assembly directory writes that directory.
#
# The output is written 0600 because it holds the pod's key.
#
# Knobs, environment only:
#
#   BRENN_POD_DIR          the brenn-pod checkout (default: ../brenn-pod)

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

usage="usage: ${prog} <pod-id> <speech-config> <out>"
[ "$#" -eq 3 ] || die "$usage"
pod_id=$1
config=$2
out=$3
# An empty argument is an unset variable that expanded inside quotes. It
# satisfies the count, so it is refused here in this tool's vocabulary.
for arg in "$pod_id" "$config" "$out"; do
	[ -n "$arg" ] || die "$usage"
done

writer="${brenn_pod_dir}/firmware/tools/provision-reachy-pod.sh"

# Both refusals are asked here, in this repo's vocabulary, rather than left to
# the other repo's script, whose messages speak to its own operators.
[ -f "$config" ] ||
	die "there is no speech configuration at ${config}, so the pod's half of the link cannot be composed." \
		"That file is the assembly directory's, and it is what both halves of the link come from."

mapfile -t checkout_remedy < <(knob_remedy BRENN_POD_DIR "<path to brenn-pod>" motion-build)
[ -x "$writer" ] ||
	die "there is no ${writer}, so the pod's link cannot be composed." \
		"A build that stages a speech configuration composes the audio device's link with" \
		"brenn-pod's own writer, from the brenn-pod checkout; REACHY_POD_BINARY names a" \
		"binary and does not excuse the checkout. The default is a sibling of this checkout." \
		"${checkout_remedy[@]}"

# Absolute, because the writer resolves a relative path against its own
# repository root.
config_abs=$(cd -- "$(dirname -- "$config")" && pwd)/$(basename -- "$config")

# Removed first so a pre-existing file's mode cannot survive the umask.
umask 077
rm -f -- "$out"
echo "${prog}: composing ${pod_id}'s link from ${config_abs}" >&2
if ! "$writer" --on-unit --emit "$pod_id" "$config_abs" >"$out"; then
	rm -f -- "$out"
	die "brenn-pod's writer refused to compose ${pod_id}'s link; its own message is above."
fi
if ! [ -s "$out" ]; then
	rm -f -- "$out"
	die "brenn-pod's writer reported success and composed nothing for ${pod_id}."
fi
