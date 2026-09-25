#!/usr/bin/env bash
#
# tools/pod-conf.test.sh — self-check for pod-conf.sh.
#
# The subject's whole job is one invocation of another repository's writer, so
# what is pinned here is that invocation — which script, with which argument
# vector, the configuration absolute — and what it leaves behind: the writer's
# stdout, byte for byte, in a file only its owner can read. The writer is
# stubbed at the path the subject looks for it; nothing here reaches a device,
# a network, or a real brenn-pod checkout.
#
# The refusals are the other half. Each has to speak in this repository's
# vocabulary, and a refusal of any kind leaves no output file: a half-composed
# link staged into a payload is a pod that parks forever.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and the writer it finds in brenn-pod
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/host"
cp -- "${script_dir}/pod-conf.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/pod-conf.sh"

# The brenn-pod checkout, as a sibling of this one — the layout the default
# assumes and the one an operator has.
pod="${work}/brenn-pod"
mkdir -p -- "${pod}/firmware/tools"

# The assembly directory: the operator's own configuration, outside either tree.
assembly="${work}/assembly"
mkdir -p -- "$assembly"
printf 'listen_addr = "127.0.0.1:7380"\npod_psk_file = "pod-psk.toml"\n' \
	>"${assembly}/speech.toml"

outdir="${work}/out"
mkdir -p -- "$outdir"
out="${outdir}/audio.conf"

export CALLS="${work}/calls"

# The writer, recording its argv on one line. Under `--emit` its stdout is the
# file and its status lines go to stderr, which is the split the subject relies
# on.
cat >"${pod}/firmware/tools/provision-reachy-pod.sh" <<'STUB'
#!/usr/bin/env bash
printf 'writer %s\n' "$*" >>"$CALLS"
if [ -n "${WRITER_STDERR:-}" ]; then
	printf '%s\n' "$WRITER_STDERR" >&2
fi
if [ -n "${WRITER_EMPTY:-}" ]; then
	exit 0
fi
printf 'ADDR=127.0.0.1:7380\nPSK=stubkey\n'
exit "${WRITER_STATUS:-0}"
STUB
chmod +x -- "${pod}/firmware/tools/provision-reachy-pod.sh"

# What the stub prints, for a byte-for-byte compare.
expected="${work}/expected"
printf 'ADDR=127.0.0.1:7380\nPSK=stubkey\n' >"$expected"

calls() { cat -- "$CALLS" 2>/dev/null || true; }

# One invocation of the subject, with its output and status captured so a
# refusal is a case rather than the end of this run.
compose() {
	: >"$CALLS"
	local out status=0
	out=$(cd -- "$repo" && "$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

mode_of() { stat -c %a -- "$1"; }

same_bytes() {
	local label=$1 want=$2 got=$3
	if cmp -s -- "$want" "$got"; then
		pass "$label"
	else
		fail "$label" "expected the bytes of ${want} at ${got}"
	fi
}

export BRENN_POD_DIR="$pod"

# ---------------------------------------------------------------------------
# The invocation
# ---------------------------------------------------------------------------

result=$(compose reachy00 "${assembly}/speech.toml" "$out")
assert_status "a composition succeeds" 0 "$(status_of "$result")"
# The arrangement, not the place the command runs: a payload's voice host is on
# the unit by definition, so the opt-in that makes a loopback listen address and
# a config-relative key table the right shapes is derivable and never typed.
assert_eq "and calls the writer on-unit, to stdout, for the pod it was given" \
	"writer --on-unit --emit reachy00 ${assembly}/speech.toml" "$(calls)"
same_bytes "the file is the writer's stdout, byte for byte" "$expected" "$out"
assert_eq "and only its owner can read it" 600 "$(mode_of "$out")"

# A relative configuration is resolved here, because the writer resolves a
# relative path against its own repository root.
printf 'listen_addr = "127.0.0.1:7380"\n' >"${repo}/host/speech.toml"
result=$(compose reachy00 host/speech.toml "$out")
assert_status "a relative configuration is composed" 0 "$(status_of "$result")"
assert_eq "and reaches the writer absolute" \
	"writer --on-unit --emit reachy00 ${repo}/host/speech.toml" "$(calls)"
rm -f -- "${repo}/host/speech.toml"

# A file already at the output keeps neither its content nor its mode.
printf 'stale\n' >"$out"
chmod 0644 -- "$out"
result=$(compose reachy00 "${assembly}/speech.toml" "$out")
assert_status "an existing output is replaced" 0 "$(status_of "$result")"
same_bytes "with the new content" "$expected" "$out"
assert_eq "and at the owner-only mode, not the old one" 600 "$(mode_of "$out")"

# The writer's status lines are on its stderr and reach the operator; they are
# never in the file.
result=$(WRITER_STDERR="reachy00 composed" compose reachy00 "${assembly}/speech.toml" "$out")
assert_contains "the writer's own report reaches the operator" \
	"$(output_of "$result")" "reachy00 composed"
same_bytes "and stays out of the file" "$expected" "$out"

# ---------------------------------------------------------------------------
# The writer refusing, or composing nothing
# ---------------------------------------------------------------------------

result=$(WRITER_STATUS=3 WRITER_STDERR="the writer's reason" \
	compose reachy00 "${assembly}/speech.toml" "$out")
assert_status "a refusal from the writer stops this one" 1 "$(status_of "$result")"
assert_contains "saying so" "$(output_of "$result")" "refused to compose reachy00's link"
assert_contains "after the writer's own reason" "$(output_of "$result")" "the writer's reason"
assert_no_file "and leaving no half-composed file" "$out"

result=$(WRITER_EMPTY=1 compose reachy00 "${assembly}/speech.toml" "$out")
assert_status "a writer that composed nothing is refused" 1 "$(status_of "$result")"
assert_contains "saying so" "$(output_of "$result")" "composed nothing"
assert_no_file "and leaving no empty file" "$out"

# ---------------------------------------------------------------------------
# The two refusals of this repository's own
# ---------------------------------------------------------------------------

result=$(compose reachy00 "${assembly}/absent.toml" "$out")
assert_status "a configuration that is not there refuses" 1 "$(status_of "$result")"
assert_contains "naming the file it looked for" "$(output_of "$result")" \
	"${assembly}/absent.toml"
assert_contains "and what cannot be done without it" "$(output_of "$result")" \
	"the pod's half of the link cannot be composed"
assert_lacks "and the writer is never asked" "$(calls)" "writer"
assert_no_file "nor is a file left" "$out"

result=$(BRENN_POD_DIR="${work}/nowhere" compose reachy00 "${assembly}/speech.toml" "$out")
assert_status "no writer in the brenn-pod checkout refuses" 1 "$(status_of "$result")"
assert_contains "naming where it looked" "$(output_of "$result")" \
	"${work}/nowhere/firmware/tools/provision-reachy-pod.sh"
assert_contains "and the knob that moves it" "$(output_of "$result")" "BRENN_POD_DIR"
# The binary's own escape hatch names a file, not the checkout the writer is in.
assert_contains "and that the binary's escape hatch does not excuse it" \
	"$(output_of "$result")" "REACHY_POD_BINARY"
assert_contains "in the goal that runs this" "$(output_of "$result")" "make motion-build"
assert_lacks "and nothing is run" "$(calls)" "writer"

# ---------------------------------------------------------------------------
# The arity
# ---------------------------------------------------------------------------

for args in "" "reachy00 ${assembly}/speech.toml" \
	"reachy00 ${assembly}/speech.toml ${out} extra"; do
	# shellcheck disable=SC2086 # the words are the case
	result=$(compose $args)
	assert_status "arguments '${args}' refuse" 1 "$(status_of "$result")"
	assert_contains "with the usage line" "$(output_of "$result")" \
		"<pod-id> <speech-config> <out>"
	assert_lacks "and the writer is never asked" "$(calls)" "writer"
done

# One argument that is empty is an unset variable expanded inside quotes. It
# satisfies the count, so it is refused here in this tool's vocabulary.
result=$(compose "" "${assembly}/speech.toml" "$out")
assert_status "an empty pod id refuses" 1 "$(status_of "$result")"
assert_contains "with the usage line too" "$(output_of "$result")" \
	"<pod-id> <speech-config> <out>"
assert_lacks "and reaches the writer not at all" "$(calls)" "writer"

tally
