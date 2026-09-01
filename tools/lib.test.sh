#!/usr/bin/env bash
#
# tools/lib.test.sh — self-check for the shared prelude: expand_home, and the
# three helpers that read where Bazel put what it built.
#
# expand_home decides two things nothing else re-checks: the value the CI
# workflow writes into BAZELISK_HOME, and whether two spellings of one directory
# compare equal inside ci-assert-bazel-caches.sh. Its coverage through that
# script is indirect, and the function is small enough to look simplifiable —
# which is how the tilde-in-a-variable trick and the HOME refusal get removed by
# someone tidying up. These cases are direct, so such an edit is red here.
#
# HOME is redirected to a fixed string, so the expectations are exact and no case
# depends on the machine. The Bazel helpers run against a stub bazel in this
# run's own directory and resolve paths against this checkout, which they only
# ever read; nothing here builds anything or reaches a network.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"


# The subject, sourced into this shell. `prog` and `repo_root` come with it and
# are what its own die() reports; nothing here depends on their values.
# shellcheck source=lib.sh
. "${script_dir}/lib.sh"

# The prelude's own path, taken before the tilde cases re-source lib.sh in a
# subshell: after that, the linter reads every mention of `repo_root` as one that
# might have been changed in there.
lib_path="${repo_root}/tools/lib.sh"

HOME=/home/tester
export HOME

# The leading-tilde marker, assembled from a variable for the reason the subject
# assembles one: a quoted literal reads to the linter as a tilde that failed to
# expand. Named for what it means rather than what it is, because the linter also
# reads a variable named after the character as one that leaked out of a subshell.
home_ref='~'

assert_eq "a bare tilde is HOME" "/home/tester" "$(expand_home "$home_ref")"
assert_eq "a tilde-rooted path expands" "/home/tester/.cache/bazelisk" \
	"$(expand_home "${home_ref}/.cache/bazelisk")"
assert_eq "a trailing slash survives" "/home/tester/.cache/" \
	"$(expand_home "${home_ref}/.cache/")"
assert_eq "an absolute path is unchanged" "/var/lib/brenn-app" \
	"$(expand_home /var/lib/brenn-app)"
assert_eq "a relative path is unchanged" "tools/lib.sh" "$(expand_home tools/lib.sh)"
assert_eq "the empty string is unchanged" "" "$(expand_home "")"

# Only a *leading* tilde is a home reference. One elsewhere in the path, and a
# user-qualified one this function deliberately does not implement, both pass
# through as themselves.
assert_eq "an embedded tilde is unchanged" "/tmp/back~up/x" "$(expand_home "/tmp/back~up/x")"
assert_eq "a user-qualified tilde is unchanged" "${home_ref}other/x" \
	"$(expand_home "${home_ref}other/x")"

# An unset HOME must stop rather than yield /.cache/..., which is what the
# workflow would otherwise export as bazelisk's download directory. The subject
# is re-sourced in a subshell so the refusal's exit does not end this run.
refusal() {
	local arg=$1 out status=0
	out=$(
		unset HOME
		# shellcheck source=lib.sh
		. "${script_dir}/lib.sh"
		expand_home "$arg" 2>&1
	) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

result=$(refusal "${home_ref}/.cache/bazelisk")
assert_eq "an unset HOME refuses a tilde path" "1" "$(sed -n '$s/^---status //p' <<<"$result")"
if grep -qF -- "HOME is unset or empty" <<<"$result"; then
	pass "the refusal says HOME is missing"
else
	fail "the refusal says HOME is missing" "in:" "$result"
fi

result=$(refusal "$home_ref")
assert_eq "an unset HOME refuses a bare tilde" "1" "$(sed -n '$s/^---status //p' <<<"$result")"

result=$(refusal /var/lib/brenn-app)
assert_eq "an unset HOME does not affect an absolute path" "0" \
	"$(sed -n '$s/^---status //p' <<<"$result")"

# ---------------------------------------------------------------------------
# The Bazel output helpers
# ---------------------------------------------------------------------------
#
# Both device build scripts read where Bazel put things through these three, and
# the three refusals are the deliverable: they are what an operator reads in the
# middle of a hardware session, and each names the thing to go and look at. Both
# scripts' own suites see them through a subject that also builds a payload;
# these cases drive them directly, so an edit to the prose or to the exact-match
# rule is red here whichever script is being worked on.
#
# `bazel` and `build_flags` are the sourcing script's to set, so this file sets
# them, as build-bench.sh and build-motion.sh do.
stub="${work}/bazel"
cat >"$stub" <<'STUB'
#!/usr/bin/env bash
[ -z "${STUB_STATUS:-}" ] || exit "$STUB_STATUS"
printf '%s' "${STUB_OUTPUT:-}"
STUB
chmod 0755 -- "$stub"

bazel=$stub
build_flags=(--config=device)

# One helper's run, output and status as one string, so a case can assert on
# both: each of them refuses by calling die, whose exit this subshell contains.
attempt() {
	local out status=0
	out=$("$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

STUB_OUTPUT=$'bazel-out/bin/one\nbazel-out/bin/two\n'
export STUB_OUTPUT STUB_STATUS=
result=$(attempt bazel_files //some:target)
assert_status "a cquery that answers is passed through" 0 "$(status_of "$result")"
assert_eq "and the whole listing comes back" $'bazel-out/bin/one\nbazel-out/bin/two' \
	"$(output_of "$result")"

STUB_STATUS=1
result=$(attempt bazel_files //some:target)
assert_status "a cquery that fails refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names the expression" "$(output_of "$result")" \
	"bazel cannot name the outputs of //some:target"
STUB_STATUS=

STUB_OUTPUT=""
result=$(attempt bazel_files //some:target)
assert_status "a cquery that answers nothing refuses" 1 "$(status_of "$result")"
assert_contains "and says no output was named" "$(output_of "$result")" \
	"bazel named no output file for //some:target"

# Resolution is against the real checkout, since `repo_root` is where lib.sh
# lives: a path Bazel could plausibly have named and a path nothing is at.
result=$(attempt bazel_resolve tools/lib.sh)
assert_status "a path the tree has resolves" 0 "$(status_of "$result")"
assert_eq "and comes back absolute" "$lib_path" "$(output_of "$result")"

result=$(attempt bazel_resolve bazel-out/bin/not-there)
assert_status "a path nothing is at refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the path" "$(output_of "$result")" \
	"the build named bazel-out/bin/not-there and no file is there"
assert_contains "and both flags that explain it" "$(output_of "$result")" "--symlink_prefix"
assert_contains "and the other one" "$(output_of "$result")" \
	"--noexperimental_convenience_symlinks"

# The exact-basename rule, with a decoy whose name contains the wanted one: a
# listing carries the sources of what was built as well as its outputs, so a
# substring match stages a file nobody asked for.
listing=$'tools/xlib.sh\ntools/lib.sh\ntools/test-lib.sh'
result=$(attempt bazel_named_in "$listing" lib.sh)
assert_status "a basename the listing carries resolves" 0 "$(status_of "$result")"
assert_eq "and it is the exact match, not the one containing it" \
	"$lib_path" "$(output_of "$result")"

result=$(attempt bazel_named_in "$listing" nothing.tachyon)
assert_status "a basename the listing does not carry refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names it" "$(output_of "$result")" \
	"the build emits no nothing.tachyon"

# The pinion agreement over the files that actually ship it. The checker's own
# behaviour on drift is pinned in build-motion.test.sh against a synthesized
# file in a temporary tree; what is asserted here is the claim that matters at a
# bench — that the two logger configurations in *this* checkout state the
# flagless defaults. The device path runs this check from build-motion.sh and the
# host path from host-motion-run.sh, and neither is a target `make check` runs,
# so without these two cases an edit to either file is green in CI and refused
# only in front of a powered unit.
for shipped in cogs/robot_logger.textproto cogs/host_logger.textproto; do
	result=$(attempt check_pinion_defaults "$shipped")
	assert_status "${shipped} states the flagless pinion defaults" 0 \
		"$(status_of "$result")"
done

result=$(attempt check_pinion_defaults cogs/no_such_logger.textproto)
assert_status "a logger configuration the tree does not have refuses" 1 \
	"$(status_of "$result")"
assert_contains "and the refusal names the file it wanted" "$(output_of "$result")" \
	"cogs/no_such_logger.textproto"

# ---------------------------------------------------------------------------
# The speech configuration reader
# ---------------------------------------------------------------------------
#
# Two scripts read the same four values out of a file neither of them writes:
# the build stages the credential files the configuration names, and the push
# asks whether any of them has moved on. Every shape this gets wrong is silent
# and expensive — a path truncated at a `#` is a plausible wrong file, a key
# read as absent is a credential the payload does not carry, and a `url` read
# out of the wrong table is a preflight against the wrong service. So the
# accept/refuse table is driven here directly, rather than through either
# script's own suite where a value only shows up as a file that did or did not
# get staged.

speech_toml="${work}/speech.toml"
cat >"$speech_toml" <<'TOML'
# the pod's side of the link
listen_addr = "127.0.0.1:7380"
pod_psk_file = "secrets/pod-psk.toml"   # beside the configuration
ident = 'reachy00'
bare_key = plain

[stt]
url = "http://speaches.example:8000"

[tts]
url = "http://speaches.example:8001"

[brenn.bridge]
token_file = "secrets/remote.token"
TOML

assert_eq "a top-level quoted value reads" "secrets/pod-psk.toml" \
	"$(toml_table_value "$speech_toml" "" pod_psk_file)"
assert_eq "a trailing comment is not part of it" "127.0.0.1:7380" \
	"$(toml_table_value "$speech_toml" "" listen_addr)"
assert_eq "a single-quoted value reads" "reachy00" \
	"$(toml_table_value "$speech_toml" "" ident)"
assert_eq "a bare value reads" "plain" \
	"$(toml_table_value "$speech_toml" "" bare_key)"
assert_eq "a dotted table's key reads" "secrets/remote.token" \
	"$(toml_table_value "$speech_toml" brenn.bridge token_file)"

# The scoping, in both directions: the two urls are the same key in different
# tables, and a top-level read must not answer with either of them.
assert_eq "[stt] url is the stt one" "http://speaches.example:8000" \
	"$(toml_table_value "$speech_toml" stt url)"
assert_eq "[tts] url is the tts one" "http://speaches.example:8001" \
	"$(toml_table_value "$speech_toml" tts url)"
assert_eq "a key under a table does not answer at the top level" "" \
	"$(toml_table_value "$speech_toml" "" url)"
assert_eq "a top-level key does not answer under a table" "" \
	"$(toml_table_value "$speech_toml" stt pod_psk_file)"
assert_eq "a key the file does not state reads empty" "" \
	"$(toml_table_value "$speech_toml" "" nothing_here)"
assert_eq "a table the file does not have reads empty" "" \
	"$(toml_table_value "$speech_toml" brain clip)"

# A `#` inside a quoted value is part of the value: truncating there names a
# plausible wrong file, which is the failure this reader exists to not have.
printf 'pod_psk_file = "secrets/pod#1.toml"\n' >"${work}/hash.toml"
assert_eq "a hash inside a quoted value survives" "secrets/pod#1.toml" \
	"$(toml_table_value "${work}/hash.toml" "" pod_psk_file)"

# The six refusals. Each has a reading this would otherwise get wrong without
# saying so.
printf 'pod_psk_file = "secrets/pod-psk.toml\n' >"${work}/unclosed.toml"
result=$(attempt toml_table_value "${work}/unclosed.toml" "" pod_psk_file)
assert_status "a value whose quoting does not close refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says which key" "$(output_of "$result")" "pod_psk_file"
assert_contains "and what it wanted instead" "$(output_of "$result")" \
	"the value's quoting does not close"

printf 'pod_psk_file = "a.toml"\npod_psk_file = "b.toml"\n' >"${work}/twice.toml"
result=$(attempt toml_table_value "${work}/twice.toml" "" pod_psk_file)
assert_status "a key stated twice in one table refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says it will not choose" "$(output_of "$result")" \
	"not this reader's to decide"

printf '[stt\nurl = "http://x"\n' >"${work}/header.toml"
result=$(attempt toml_table_value "${work}/header.toml" "" url)
assert_status "a table header this cannot parse refuses" 1 "$(status_of "$result")"
assert_contains "and says every key after it would be misfiled" "$(output_of "$result")" \
	"filed under the wrong table"

# The three legal spellings this reader does not descend into. Each of them
# would otherwise read as *absent* — a build that stages no credential and says
# nothing about it — which is worse than either a value or a refusal.
printf '[[server]]\nurl = "http://x"\n' >"${work}/array.toml"
result=$(attempt toml_table_value "${work}/array.toml" "" pod_psk_file)
assert_status "an array-of-tables header refuses" 1 "$(status_of "$result")"
assert_contains "and says its keys are under a table nothing asked for" \
	"$(output_of "$result")" "a table nothing asked for"

printf 'brenn.bridge.token_file = "secrets/remote.token"\n' >"${work}/dotted.toml"
result=$(attempt toml_table_value "${work}/dotted.toml" "" pod_psk_file)
assert_status "a dotted key in the table being read refuses" 1 "$(status_of "$result")"
assert_contains "and says it would otherwise read as absent" "$(output_of "$result")" \
	"would read as absent"

printf '[brenn]\nbridge = { token_file = "secrets/remote.token" }\n' >"${work}/inline.toml"
result=$(attempt toml_table_value "${work}/inline.toml" brenn nothing_here)
assert_status "an inline table in the table being read refuses" 1 "$(status_of "$result")"
assert_contains "and says its keys are all on one line" "$(output_of "$result")" \
	"all on one line"

# A multiline string is refused wherever in the file it is, because what it
# costs is the scope this walk keeps: a bracketed line in its body re-scopes
# every key after it, and the key being read then comes back absent with a
# staged payload carrying no credential and nothing said about it.
printf 'pod_psk_file = "secrets/pod-psk.toml"\nfailure_message = """\n[not] a table\nurl = "x"\n"""\n' \
	>"${work}/multiline.toml"
result=$(attempt toml_table_value "${work}/multiline.toml" "" pod_psk_file)
assert_status "a multiline basic string refuses" 1 "$(status_of "$result")"
assert_contains "and says this reader takes one line per value" "$(output_of "$result")" \
	"one line per value"

printf "[voice]\nfailure_message = '''\n[not] a table\n'''\n\n[stt]\nurl = \"http://x\"\n" \
	>"${work}/multiline-literal.toml"
result=$(attempt toml_table_value "${work}/multiline-literal.toml" stt url)
assert_status "a multiline literal string under another table refuses too" 1 \
	"$(status_of "$result")"
assert_contains "and says why the body is the problem" "$(output_of "$result")" \
	"read as TOML"

# A dotted key outside the table being read is not this read's problem: the
# refusal is scoped, so a site's own nested tables elsewhere in the file do not
# refuse a read that could never have seen them.
printf '[brain]\nsome.nested = "x"\n\n[stt]\nurl = "http://x"\n' >"${work}/elsewhere.toml"
assert_eq "a dotted key under another table does not refuse this read" "http://x" \
	"$(toml_table_value "${work}/elsewhere.toml" stt url)"

# The same key in two different tables is not a duplicate: that is the shape the
# stt and tts urls have, and refusing it would refuse every real configuration.
result=$(attempt toml_table_value "$speech_toml" stt url)
assert_status "the same key under two tables is not a duplicate" 0 "$(status_of "$result")"

# ---------------------------------------------------------------------------
# The credential paths, and what a payload cannot carry
# ---------------------------------------------------------------------------

assert_eq "both credential fields come back, keyed, with the source beside the configuration" \
	"pod_psk_file	secrets/pod-psk.toml	${work}/secrets/pod-psk.toml
token_file	secrets/remote.token	${work}/secrets/remote.token" \
	"$(speech_credential_paths "$speech_toml")"

# A configuration with no [brenn.bridge] is a voiced, bus-less pipeline: legal,
# and it stages no token.
printf 'pod_psk_file = "pod-psk.toml"\n' >"${work}/busless.toml"
assert_eq "a configuration with no bridge names only the key table" \
	"pod_psk_file	pod-psk.toml	${work}/pod-psk.toml" \
	"$(speech_credential_paths "${work}/busless.toml")"

# A configuration that is not there names nothing: a payload built without one
# is the ordinary case, and the build refuses a *named* missing one on its own.
assert_eq "a configuration that is not there names nothing" "" \
	"$(speech_credential_paths "${work}/no-such-config.toml")"

printf 'pod_psk_file = "/etc/brenn/pod-psk.toml"\n' >"${work}/absolute.toml"
result=$(attempt speech_credential_paths "${work}/absolute.toml")
assert_status "an absolute credential path refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says the payload carries its own" \
	"$(output_of "$result")" "the payload carries its own credentials"
assert_contains "and says what it would do on the unit" "$(output_of "$result")" \
	"names nothing on the unit"

printf 'pod_psk_file = "../pod-psk.toml"\n' >"${work}/climbing.toml"
result=$(attempt speech_credential_paths "${work}/climbing.toml")
assert_status "a credential path that climbs out of the payload refuses" 1 \
	"$(status_of "$result")"
assert_contains "and the refusal says so" "$(output_of "$result")" \
	"climbs out of the payload"

printf 'pod_psk_file = "secrets/"\n' >"${work}/directory.toml"
result=$(attempt speech_credential_paths "${work}/directory.toml")
assert_status "a credential path naming a directory refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says it names no file" "$(output_of "$result")" \
	"names no file"

# A `.` component names a payload member under a spelling the build's collision
# check does not match, and the filesystem resolves what the compare did not: the
# credential would be installed over the launcher config the build staged a
# moment earlier, at 0600, with the build reporting success.
printf 'pod_psk_file = "./robotcpu.textproto"\n' >"${work}/dot-component.toml"
result=$(attempt speech_credential_paths "${work}/dot-component.toml")
assert_status "a credential path with a . component refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says what the compare would miss" "$(output_of "$result")" \
	"a . or an empty component"

printf 'pod_psk_file = "models//oww/melspectrogram.onnx"\n' >"${work}/empty-component.toml"
result=$(attempt speech_credential_paths "${work}/empty-component.toml")
assert_status "a credential path with a doubled slash refuses" 1 "$(status_of "$result")"
assert_contains "with the same refusal" "$(output_of "$result")" \
	"a . or an empty component"

# A `.` inside a name is not a component: refusing it would refuse every file
# with an extension.
printf 'pod_psk_file = "secrets/pod-psk.toml"\n' >"${work}/plain-dot.toml"
assert_eq "a dot inside a name is not a component" \
	"pod_psk_file	secrets/pod-psk.toml	${work}/secrets/pod-psk.toml" \
	"$(speech_credential_paths "${work}/plain-dot.toml")"

# A `..` that is part of a name is not a climb: refusing it would be this
# reader deciding what a site may call its own files.
printf 'pod_psk_file = "secrets/pod..psk.toml"\n' >"${work}/dots.toml"
assert_eq "a doubled dot inside a name is not a climb" \
	"pod_psk_file	secrets/pod..psk.toml	${work}/secrets/pod..psk.toml" \
	"$(speech_credential_paths "${work}/dots.toml")"

# ---------------------------------------------------------------------------
# The service endpoints, and what a remote command can carry
# ---------------------------------------------------------------------------
#
# These are asked of the unit before a speech run starts, so each value is
# pasted into a command run there as root. A value that is not a URL means
# something else at that site, which is why it is refused where it is read.

assert_eq "both services come back, keyed by their table" \
	$'stt\thttp://speaches.example:8000\ntts\thttp://speaches.example:8001' \
	"$(speech_service_urls "$speech_toml")"

printf '[stt]\nurl = "https://speaches.example/v1"\n' >"${work}/stt-only.toml"
assert_eq "a configuration with one service names one" \
	$'stt\thttps://speaches.example/v1' \
	"$(speech_service_urls "${work}/stt-only.toml")"

printf 'pod_psk_file = "pod-psk.toml"\n' >"${work}/no-services.toml"
assert_eq "a configuration with neither table names nothing" "" \
	"$(speech_service_urls "${work}/no-services.toml")"

assert_eq "a configuration that is not there names no service either" "" \
	"$(speech_service_urls "${work}/no-such-config.toml")"

printf '[tts]\nurl = "speaches.example:8001"\n' >"${work}/schemeless.toml"
result=$(attempt speech_service_urls "${work}/schemeless.toml")
assert_status "an endpoint that is not a URL refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal says what it wanted" "$(output_of "$result")" \
	"not an http or https URL"

# A trailing slash comes off here, because the caller appends a probe path and
# `//v1/models` is a 404 that would be read as an unreachable robot.
printf '[stt]\nurl = "http://speaches.example:8000/"\n' >"${work}/slash.toml"
assert_eq "a trailing slash is taken off the endpoint" \
	$'stt\thttp://speaches.example:8000' \
	"$(speech_service_urls "${work}/slash.toml")"

printf '[stt]\nurl = "http://x:8000; rm -rf /"\n' >"${work}/injected.toml"
result=$(attempt speech_service_urls "${work}/injected.toml")
assert_status "an endpoint carrying a command refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names the table and the value" "$(output_of "$result")" \
	"[stt] url in ${work}/injected.toml is 'http://x:8000; rm -rf /'"
assert_contains "and says what is accepted" "$(output_of "$result")" \
	"[A-Za-z0-9:/._~%-]"

# ---------------------------------------------------------------------------
# Where the other repository is
# ---------------------------------------------------------------------------
#
# One knob for one physical fact: BRENN_POD_DIR names the brenn-pod checkout,
# and both things this repo takes out of it — the prebuilt audio binary the
# payload stages and the provisioning a speech run invokes — resolve under it.
# REACHY_POD_BINARY still wins for the binary alone, because it names a file
# rather than a repository: an artifact copied out of a build somewhere else has
# no checkout around it.
#
# Re-sourced in a subshell per case, because both values are assigned at source
# time.
# The checkout's own root, taken off the path captured before any subshell
# re-sourced the prelude: `repo_root` itself reads to the linter as a value that
# may have escaped one of them.
checkout=${lib_path%/tools/lib.sh}

pod_paths() {
	(
		# shellcheck source=lib.sh
		. "${script_dir}/lib.sh"
		printf '%s\n%s\n' "$brenn_pod_dir" "$pod_binary"
	)
}

assert_eq "with no knob set the checkout is this one's sibling" \
	"${checkout}/../brenn-pod" "$(pod_paths | sed -n 1p)"
assert_eq "and the audio binary is the one that build leaves there" \
	"${checkout}/../brenn-pod/firmware/target/reachy-pod/payload/reachy-pod" \
	"$(pod_paths | sed -n 2p)"

assert_eq "an absolute BRENN_POD_DIR is taken as it stands" "/elsewhere/brenn-pod" \
	"$(BRENN_POD_DIR=/elsewhere/brenn-pod pod_paths | sed -n 1p)"
assert_eq "and the audio binary follows it, with no second knob to set" \
	"/elsewhere/brenn-pod/firmware/target/reachy-pod/payload/reachy-pod" \
	"$(BRENN_POD_DIR=/elsewhere/brenn-pod pod_paths | sed -n 2p)"

# The Makefile's own default is spelled relatively and exported, so a script run
# by hand from a subdirectory has to resolve it the way a recipe's child does:
# against this repository's root, never against the caller's directory.
assert_eq "a relative BRENN_POD_DIR is relative to this checkout" \
	"${checkout}/../pod-elsewhere" \
	"$(BRENN_POD_DIR=../pod-elsewhere pod_paths | sed -n 1p)"

assert_eq "REACHY_POD_BINARY still names the artifact by itself" \
	"/tmp/reachy-pod" \
	"$(REACHY_POD_BINARY=/tmp/reachy-pod pod_paths | sed -n 2p)"
assert_eq "and wins over a checkout named beside it" "/tmp/reachy-pod" \
	"$(BRENN_POD_DIR=/elsewhere/brenn-pod REACHY_POD_BINARY=/tmp/reachy-pod \
		pod_paths | sed -n 2p)"

# ---------------------------------------------------------------------------
# knob_remedy
# ---------------------------------------------------------------------------
assert_eq "the remedy is the two ways to set one knob" \
	"$(printf '%s\n' "Name it for this invocation:" \
		"    make speech-run BRENN_POD_DIR=<path to brenn-pod>" \
		"or once, in the gitignored .local/reachy.conf:" \
		"    BRENN_POD_DIR ?= <path to brenn-pod>")" \
	"$(knob_remedy BRENN_POD_DIR "<path to brenn-pod>")"
assert_eq "and the goal it names is the caller's" \
	"    make motion-build REACHY_SPEECH_CONFIG=<assembly>/speech.toml" \
	"$(knob_remedy REACHY_SPEECH_CONFIG "<assembly>/speech.toml" motion-build |
		sed -n 2p)"

tally
