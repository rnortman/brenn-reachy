#!/usr/bin/env bash
#
# tools/provision-speech.test.sh — self-check for provision-speech.sh.
#
# The subject's whole job is one invocation of another repository's make target,
# so what is pinned here is that invocation: which target, in which checkout,
# with which values, and on the sub-make's command line rather than in its
# environment. `make` is stubbed on PATH and records what it was asked for;
# nothing here reaches a device, a network, or a real brenn-pod checkout.
#
# The two refusals are the other half. Each is a thing an operator meets while
# already stopped, and each has to speak in this repository's vocabulary: the
# other repository's script refuses the same conditions in words that name its
# own conf file and its own command line, which are right for a session there
# and wrong for a `make speech-run` here.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and the stub it finds on PATH
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/host"
cp -- "${script_dir}/provision-speech.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/provision-speech.sh"

# The brenn-pod checkout, as a sibling of this one — the layout the default
# assumes and the one an operator has.
pod="${work}/brenn-pod"
mkdir -p -- "${pod}/firmware"
: >"${pod}/firmware/Makefile"

# The assembly directory: the operator's own configuration, outside either tree.
assembly="${work}/assembly"
mkdir -p -- "$assembly"
printf 'listen_addr = "127.0.0.1:8765"\npod_psk_file = "pod-psk.toml"\n' \
	>"${assembly}/speech.toml"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

export CALLS="${work}/calls"
export MAKE_STATUS=0

# The sub-make, recording its whole argv on one line and its inherited
# environment on the next: the requirement is that the two values arrive as
# command-line overrides, which is what a `.local/reachy.conf` in the other
# checkout cannot beat.
cat >"${stubs}/make" <<'STUB'
#!/usr/bin/env bash
printf 'make %s\n' "$*" >>"$CALLS"
printf 'env MAKEFLAGS=[%s] SPEECH_CONFIG=[%s]\n' \
	"${MAKEFLAGS-unset}" "${SPEECH_CONFIG-unset}" >>"$CALLS"
exit "${MAKE_STATUS:-0}"
STUB
chmod +x -- "${stubs}/make"

calls() { cat -- "$CALLS" 2>/dev/null || true; }

# One invocation of the subject, with its output and status captured so a
# refusal is a case rather than the end of this run.
provision() {
	: >"$CALLS"
	local out status=0
	out=$(cd -- "$repo" && "$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }

# ---------------------------------------------------------------------------
# The invocation
# ---------------------------------------------------------------------------

export BRENN_POD_DIR="$pod"
export REACHY_SPEECH_CONFIG="${assembly}/speech.toml"

result=$(provision reachy00)
assert_status "a provisioning run succeeds" 0 "$(status_of "$result")"
assert_contains "and calls the other repository's provisioning target" "$(calls)" \
	"-C ${pod}/firmware reachy-provision"
# The arrangement, not the place the command runs: a speech run is by definition
# a voice host on the unit, so the opt-in that makes a loopback listen address
# and a config-relative key table the right shapes is derivable and never typed.
assert_contains "with the on-unit arrangement it is by definition" "$(calls)" \
	"ON_UNIT=1"
assert_contains "the configuration, absolute" "$(calls)" \
	"SPEECH_CONFIG=${assembly}/speech.toml"
assert_contains "and the unit this repository was told about" "$(calls)" \
	"REACHY_HOST=reachy00"
# Command line, not environment. The other checkout carries a
# `firmware/.local/reachy.conf` of its own, and a file-origin value there would
# decide which unit or which configuration got provisioned — the two halves of
# one link derived from two different files.
assert_contains "the values ride the sub-make's command line" "$(calls)" \
	"SPEECH_CONFIG=[unset]"
# The parent make's flags are dropped: brenn-pod's build is not part of this
# one's graph, and a jobserver it cannot join is a warning bought for nothing.
assert_contains "and the parent make's flags are not inherited" "$(calls)" \
	"MAKEFLAGS=[unset]"

MAKE_STATUS=0 result=$(MAKEFLAGS=-j8 provision reachy00)
assert_contains "even when this script was itself run under one" "$(calls)" \
	"MAKEFLAGS=[unset]"

# A relative configuration is resolved here, because a relative SPEECH_CONFIG is
# resolved against the *other* repository's root and this one's default is
# relative to ours.
REACHY_SPEECH_CONFIG="host/speech.toml"
printf 'listen_addr = "127.0.0.1:8765"\n' >"${repo}/host/speech.toml"
result=$(provision reachy00)
assert_status "a relative configuration is provisioned" 0 "$(status_of "$result")"
assert_contains "and reaches the other repository absolute" "$(calls)" \
	"SPEECH_CONFIG=${repo}/host/speech.toml"
unset REACHY_SPEECH_CONFIG

# Unnamed, the configuration is this tree's gitignored default — the same
# resolution the build uses, so the file the pod's half is derived from is the
# file the payload will carry.
result=$(provision reachy00)
assert_contains "an unnamed configuration is this tree's own default" "$(calls)" \
	"SPEECH_CONFIG=${repo}/host/speech.toml"
rm -f -- "${repo}/host/speech.toml"

# The other repository refusing is this script's status: a provisioning that did
# not happen must not be followed by a build and a run against a deaf pod.
MAKE_STATUS=2
export REACHY_SPEECH_CONFIG="${assembly}/speech.toml"
result=$(provision reachy00)
assert_status "a refusal in the other repository stops this one" 2 \
	"$(status_of "$result")"
MAKE_STATUS=0

# ---------------------------------------------------------------------------
# The two refusals
# ---------------------------------------------------------------------------

export REACHY_SPEECH_CONFIG="${assembly}/absent.toml"
result=$(provision reachy00)
assert_status "a configuration that is not there refuses" 1 "$(status_of "$result")"
assert_contains "naming the file it looked for" "$(output_of "$result")" \
	"${assembly}/absent.toml"
assert_contains "and the knob that names it" "$(output_of "$result")" \
	"REACHY_SPEECH_CONFIG"
assert_contains "in the spelling the local configuration file wants" \
	"$(output_of "$result")" "REACHY_SPEECH_CONFIG ?="
# The other repository's refusal for this condition names its own conf file and
# its own command line, which is right for a session there and a dead end for
# an operator who typed `make speech-run` here.
assert_lacks "and the other repository is never asked" "$(calls)" "reachy-provision"
export REACHY_SPEECH_CONFIG="${assembly}/speech.toml"

BRENN_POD_DIR="${work}/nowhere"
result=$(provision reachy00)
assert_status "no brenn-pod checkout refuses" 1 "$(status_of "$result")"
assert_contains "naming where it looked" "$(output_of "$result")" "${work}/nowhere"
assert_contains "and the knob that moves it" "$(output_of "$result")" "BRENN_POD_DIR"
assert_lacks "and nothing is run" "$(calls)" "make"

# A checkout that is there but carries no firmware Makefile is the same refusal:
# what this needs is that repository's supported interface, not its directory.
mkdir -p -- "${work}/half/firmware"
BRENN_POD_DIR="${work}/half"
result=$(provision reachy00)
assert_status "a checkout with no firmware Makefile refuses too" 1 \
	"$(status_of "$result")"
assert_contains "and says which path it wanted" "$(output_of "$result")" \
	"${work}/half"
BRENN_POD_DIR="$pod"

# ---------------------------------------------------------------------------
# The arity
# ---------------------------------------------------------------------------

result=$(provision)
assert_status "no host refuses" 1 "$(status_of "$result")"
assert_contains "with the usage line" "$(output_of "$result")" "<host>"

# One argument that is empty is an unset variable expanded inside quotes. It
# satisfies the count, so it is refused here in this tool's vocabulary.
result=$(provision "")
assert_status "an empty host refuses" 1 "$(status_of "$result")"
assert_contains "with the usage line too" "$(output_of "$result")" "<host>"
assert_lacks "and reaches the other repository not at all" "$(calls)" "reachy-provision"

result=$(provision reachy00 extra)
assert_status "a second argument refuses" 1 "$(status_of "$result")"
assert_lacks "and provisions nothing" "$(calls)" "reachy-provision"

tally
