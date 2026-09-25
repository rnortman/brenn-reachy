#!/usr/bin/env bash
#
# tools/payload-run.test.sh — self-check for payload-run.sh, the payload's `run`.
#
# The subject is copied into a scratch payload root as `run`, beside a stub
# `simplelaunch` alone that records how it was started, under which trust
# anchor, and whether a TERM reached it, and run from that root with TMPDIR
# naming a scratch directory, the way brenn-app.service starts it. Nothing here
# touches a device or this checkout.
#
# What is worth pinning: that both log roots are emptied before anything
# starts, and that a wipe that fails, a missing configuration file, a stamp
# that cannot be copied or a config directory that cannot be made starts
# nothing; that the run's configuration and stamp land beside its records;
# that the production launcher is started on `robotcpu.textproto`; that `run`
# exits 0 whatever the launcher returned, which is what keeps the service from
# restarting onto a crashed stack; that a TERM or an INT to `run` reaches the
# launcher as TERM; and that nothing is written outside TMPDIR.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The payload root the subject runs from, and the scratch space it writes to.
# ---------------------------------------------------------------------------

payload="${work}/payload"
scratch="${work}/scratch"
mkdir -p -- "${payload}/cogs"
install -m 0755 -- "${script_dir}/payload-run.sh" "${payload}/run"

config_files=(cogs/servo_profile.textproto cogs/servo_gains.textproto cogs/mover_params.textproto)
for f in "${config_files[@]}"; do
	echo "# ${f} of this payload" >"${payload}/${f}"
done
echo 'apps {}' >"${payload}/robotcpu.textproto"
echo 'commit=abc' >"${payload}/provenance.txt"

# The launcher: records how it was started and under which trust anchor,
# answers a TERM by saying so, and then by STUB_LAUNCH either serves until
# signalled (`wait`) or exits with that status (default 0).
cat >"${payload}/simplelaunch" <<'STUB'
#!/usr/bin/env bash
echo "$*" >"${TMPDIR}/stub-launch.args"
echo "${SSL_CERT_FILE-unset}" >"${TMPDIR}/stub-launch.ssl"
trap 'touch -- "${TMPDIR}/stub-launch.term"; exit 143' TERM
echo "$PPID" >"${TMPDIR}/stub-launch.ppid"
if [ "${STUB_LAUNCH:-0}" = wait ]; then
	while :; do
		sleep 0.05
	done
fi
exit "${STUB_LAUNCH:-0}"
STUB
chmod 0755 -- "${payload}/simplelaunch"

# A fresh scratch space for each case, as a boot gives the service.
fresh() {
	if [ -d "$scratch" ]; then
		chmod -R u+w -- "$scratch"
		rm -rf -- "$scratch"
	fi
	mkdir -m 0700 -- "$scratch"
}

# The subject from the payload root, with no trust anchor unless a case names
# one. Extra arguments are environment assignments for this run.
run_payload() {
	local out status=0
	out=$(cd -- "$payload" &&
		env -u BRENN_CA_FILE -u SSL_CERT_FILE TMPDIR="$scratch" "$@" ./run 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

last_line() { output_of "$1" | tail -n 1; }

# ---------------------------------------------------------------------------
# A clean run
# ---------------------------------------------------------------------------

fresh
result=$(run_payload)
assert_status "a clean run exits 0" 0 "$(status_of "$result")"
assert_eq "and reports the launcher's own status last" \
	"run: launcher exited 0" "$(last_line "$result")"
assert_eq "the launcher is started on the production config, logging under scratch" \
	"robotcpu.textproto --logdir ${scratch}/logs/launch" \
	"$(cat -- "${scratch}/stub-launch.args")"
for f in "${config_files[@]}"; do
	if cmp -s -- "${payload}/${f}" "${scratch}/logs/motion/config/${f}"; then
		pass "${f} is beside the records at its payload-relative path"
	else
		fail "${f} is beside the records at its payload-relative path" \
			"no byte-equal copy at ${scratch}/logs/motion/config/${f}"
	fi
done
assert_eq "the stamp is beside the records" \
	"$(cat -- "${payload}/provenance.txt")" \
	"$(cat -- "${scratch}/logs/motion/provenance.txt" 2>/dev/null || true)"
assert_eq "with no trust anchor on the device, SSL_CERT_FILE is not exported" \
	unset "$(cat -- "${scratch}/stub-launch.ssl")"

# ---------------------------------------------------------------------------
# The log roots are emptied first
# ---------------------------------------------------------------------------

fresh
mkdir -p -- "${scratch}/logs/motion/1788832560362471129" "${scratch}/logs/launch"
echo old >"${scratch}/logs/motion/1788832560362471129/motion_0.olog"
echo old >"${scratch}/logs/launch/motord_0.log"
result=$(run_payload)
assert_status "a run over a previous run's records exits 0" 0 "$(status_of "$result")"
assert_no_file "and the previous run's records are gone" \
	"${scratch}/logs/motion/1788832560362471129/motion_0.olog"
assert_no_file "and so are its consoles" "${scratch}/logs/launch/motord_0.log"

label="leftovers this account cannot remove start nothing and say to reboot"
if [ "$(id -u)" -eq 0 ]; then
	echo "SKIP: ${label} (root removes anything)"
else
	fresh
	mkdir -p -- "${scratch}/logs/motion/held/keep"
	chmod 0555 -- "${scratch}/logs/motion/held"
	result=$(run_payload)
	chmod 0755 -- "${scratch}/logs/motion/held"
	assert_status "${label}: exit 0" 0 "$(status_of "$result")"
	assert_contains "${label}: the message" "$(output_of "$result")" "reboot"
	assert_no_file "${label}: no launcher" "${scratch}/stub-launch.args"
fi

# ---------------------------------------------------------------------------
# What the payload has to carry
# ---------------------------------------------------------------------------

fresh
mv -- "${payload}/cogs/servo_gains.textproto" "${work}/servo_gains.textproto"
result=$(run_payload)
mv -- "${work}/servo_gains.textproto" "${payload}/cogs/servo_gains.textproto"
assert_status "a payload missing a run configuration file exits 0" 0 "$(status_of "$result")"
assert_contains "and names the file" "$(output_of "$result")" "cogs/servo_gains.textproto"
assert_no_file "and starts no launcher" "${scratch}/stub-launch.args"

fresh
mv -- "${payload}/provenance.txt" "${work}/provenance.txt"
result=$(run_payload)
mv -- "${work}/provenance.txt" "${payload}/provenance.txt"
assert_status "a payload with no stamp still runs" 0 "$(status_of "$result")"
assert_file "and starts the launcher" "${scratch}/stub-launch.args"
assert_no_file "and leaves no stamp beside the records" "${scratch}/logs/motion/provenance.txt"

label="a stamp that cannot be copied starts nothing"
if [ "$(id -u)" -eq 0 ]; then
	echo "SKIP: ${label} (root reads anything)"
else
	fresh
	chmod 0000 -- "${payload}/provenance.txt"
	result=$(run_payload)
	chmod 0644 -- "${payload}/provenance.txt"
	assert_status "${label}: exit 0" 0 "$(status_of "$result")"
	assert_contains "${label}: the message names the stamp" "$(output_of "$result")" "provenance.txt"
	assert_no_file "${label}: no launcher" "${scratch}/stub-launch.args"
fi

# The config directory is made by run itself inside a root it just made, so
# its failure is reached through a stub mkdir that refuses that one path.
label="a config directory that cannot be made starts nothing and does not blame the payload"
mkdir -p -- "${work}/stubs"
cat >"${work}/stubs/mkdir" <<'STUB'
#!/bin/sh
case "$*" in
*config*) echo "mkdir: stub: refused $*" >&2; exit 1 ;;
esac
exec "$REAL_MKDIR" "$@"
STUB
chmod 0755 -- "${work}/stubs/mkdir"
fresh
result=$(run_payload REAL_MKDIR="$(command -v mkdir)" PATH="${work}/stubs:${PATH}")
assert_status "${label}: exit 0" 0 "$(status_of "$result")"
assert_contains "${label}: the message names the directory" "$(output_of "$result")" "cannot make"
assert_lacks "${label}: and not the payload" "$(output_of "$result")" "in the payload"
assert_no_file "${label}: no launcher" "${scratch}/stub-launch.args"

# ---------------------------------------------------------------------------
# The launcher's status is reported, never returned
# ---------------------------------------------------------------------------

fresh
result=$(run_payload STUB_LAUNCH=7)
assert_status "a launcher that fails still leaves run exiting 0" 0 "$(status_of "$result")"
assert_eq "and its status is the last line" \
	"run: launcher exited 7" "$(last_line "$result")"

# ---------------------------------------------------------------------------
# Signals reach the launcher as TERM
# ---------------------------------------------------------------------------

for sig in TERM INT; do
	fresh
	# The subject runs in the foreground, so INT is not ignored in it on
	# entry; the helper that signals it is the one in the background.
	(
		i=0
		while [ ! -s "${scratch}/stub-launch.ppid" ] && [ "$i" -lt 100 ]; do
			sleep 0.05
			i=$((i + 1))
		done
		kill -"$sig" "$(cat -- "${scratch}/stub-launch.ppid")"
	) &
	helper=$!
	result=$(run_payload STUB_LAUNCH=wait)
	wait "$helper" || true
	assert_status "a ${sig} to run: exit 0" 0 "$(status_of "$result")"
	assert_file "a ${sig} to run reaches the launcher as TERM" "${scratch}/stub-launch.term"
	assert_eq "a ${sig} to run: the launcher's own status is the last line" \
		"run: launcher exited 143" "$(last_line "$result")"
done

# ---------------------------------------------------------------------------
# The trust anchor
# ---------------------------------------------------------------------------

echo '-----BEGIN CERTIFICATE-----' >"${work}/ca.pem"
fresh
result=$(run_payload BRENN_CA_FILE="${work}/ca.pem")
assert_eq "a readable anchor is exported as SSL_CERT_FILE" \
	"${work}/ca.pem" "$(cat -- "${scratch}/stub-launch.ssl")"
fresh
result=$(run_payload BRENN_CA_FILE="${work}/nowhere.pem")
assert_eq "a missing one is not" unset "$(cat -- "${scratch}/stub-launch.ssl")"

# ---------------------------------------------------------------------------
# Nothing outside TMPDIR
# ---------------------------------------------------------------------------

fresh
before=$(find "$payload" | sort)
marker="${work}/marker"
touch -- "$marker"
touch -r "$marker" -- "$work"
result=$(run_payload)
assert_status "the clean run for the write check exits 0" 0 "$(status_of "$result")"
assert_eq "the payload root is unchanged" "$before" "$(find "$payload" | sort)"
assert_eq "nothing outside TMPDIR is newer than the run's start" "" \
	"$(find "$work" -newer "$marker" -not -path "$scratch" -not -path "${scratch}/*")"

# ---------------------------------------------------------------------------
# The script's own shape, and what it has to agree with
# ---------------------------------------------------------------------------

subject="${script_dir}/payload-run.sh"
assert_eq "the shebang is POSIX sh, which is dash on the device" \
	"#!/bin/sh" "$(head -n 1 -- "$subject")"
assert_lacks "run starts no tour" "$(cat -- "$subject")" "reachy_ask"

root=$(checkout_root)
log_root=$(sed -n 's/^log_root_dir: "\(.*\)"$/\1/p' -- "${root}/cogs/robot_logger.textproto")
assert_eq "the checked-in logger root is under the payload's scratch space" \
	"/run/brenn-app/scratch/logs/motion" "$log_root"
logs_line=$(grep '^logs=' -- "$subject")
# shellcheck disable=SC2016
assert_eq "run names its log roots under TMPDIR" 'logs="${TMPDIR:?}/logs"' "$logs_line"
assert_eq "and under the service's TMPDIR its motion root is the logger's root" \
	"$log_root" \
	"$(TMPDIR=/run/brenn-app/scratch sh -c "${logs_line}; printf '%s' \"\${logs}/motion\"")"

# ---------------------------------------------------------------------------

tally
