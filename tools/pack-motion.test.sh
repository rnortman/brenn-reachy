#!/usr/bin/env bash
#
# tools/pack-motion.test.sh — self-check for pack-motion.sh.
#
# The subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and the payload it packs is a directory this test
# staged. git is stubbed on PATH, answering the three questions the pack asks of
# the repository on knobs; tar and zstd are the real ones, because the archive's
# bytes are the thing under test. Nothing here touches a device or this
# checkout.
#
# What is worth pinning: the refusals (no payload, no `run`, no
# `cogs/robot_clk_exe`, no `zstd`, a stale payload, no build commit), and the
# archive — zstd, `run` at its root, the contract's modes and ownership, every
# timestamp the build commit's, the push's own stamp inside it, a digest that
# is the file's, and two packs of one tree giving one archive byte for byte.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

for tool in tar zstd; do
	command -v "$tool" >/dev/null 2>&1 || {
		echo "pack-motion.test.sh: no ${tool} on PATH; the pack under test needs it" >&2
		exit 1
	}
done

# `tar -tv` prints member times in the local zone.
export TZ=UTC

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and the stub it finds on PATH.
# ---------------------------------------------------------------------------

repo="${work}/repo"
payload="${repo}/target/motion-arm64/release"
archive="${repo}/target/motion-arm64/payload.tar.zst"
mkdir -p -- "${repo}/tools"
cp -- "${script_dir}/pack-motion.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/pack-motion.sh"

stubs="${work}/stubs"
mkdir -p -- "$stubs"
export PATH="${stubs}:${PATH}"

# The three questions the subject asks the repository: the newest commit to the
# workspace paths (GIT_COMMIT_TIME), the build commit's own instant
# (BUILD_COMMIT_TIME; empty is a checkout without that commit), and the stamp's
# HEAD and porcelain status (GIT_HEAD, GIT_DIRTY).
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
case " $* " in
	*" rev-parse "*)
		[ -n "${GIT_HEAD:-}" ] || exit 128
		echo "$GIT_HEAD"
		exit 0
		;;
	*" status "*)
		[ -z "${GIT_DIRTY:-}" ] || echo "$GIT_DIRTY"
		exit 0
		;;
	*" show "*)
		[ -n "${BUILD_COMMIT_TIME:-}" ] || exit 128
		echo "$BUILD_COMMIT_TIME"
		exit 0
		;;
esac
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB
chmod 0755 -- "${stubs}/git"

# Two times an hour either side of the newest commit, so "older" and "newer" are
# unambiguous whatever the filesystem's timestamp resolution is; the build
# commit is a different instant again, so a timestamp taken from the wrong one
# shows.
commit_at=1750000000
before=$((commit_at - 3600))
after=$((commit_at + 3600))
export GIT_COMMIT_TIME=$commit_at
export BUILD_COMMIT_TIME=1749990000
export BUILD_COMMIT=0123456789abcdef0123456789abcdef01234567
export GIT_HEAD=89abcdef0123456789abcdef0123456789abcdef
export GIT_DIRTY=

config_files=(cogs/servo_profile.textproto cogs/servo_gains.textproto cogs/mover_params.textproto)

# The payload as a build stages it, the executable dated `$1`.
stage_payload() {
	rm -rf -- "$payload" "$archive" "${archive}.sha256"
	mkdir -p -- "${payload}/cogs" "${payload}/host"
	printf '#!/bin/sh\nexit 0\n' >"${payload}/run"
	chmod 0755 -- "${payload}/run"
	printf 'an executable\n' >"${payload}/cogs/robot_clk_exe"
	chmod 0755 -- "${payload}/cogs/robot_clk_exe"
	local f
	for f in "${config_files[@]}"; do
		echo "# ${f}" >"${payload}/${f}"
	done
	printf 'name: "reachy00"\n' >"${payload}/host/host_params.textproto"
	chmod 0600 -- "${payload}/host/host_params.textproto"
	printf 'commit=%s\n' "$BUILD_COMMIT" >"${payload}/build-commit.txt"
	touch -d "@$1" -- "${payload}/cogs/robot_clk_exe"
}

pack() {
	local out status=0
	out=$("$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

listing() { tar -tf "$archive"; }
verbose_listing() { tar -tvf "$archive"; }
archived() { tar -xOf "$archive" "./$1"; }

# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------

rm -rf -- "$payload"
result=$(pack)
assert_status "no payload refuses" 1 "$(status_of "$result")"
assert_contains "and says how to build one" "$(output_of "$result")" "make motion-build"

stage_payload "$after"
rm -- "${payload}/run"
result=$(pack)
assert_status "a payload without run refuses" 1 "$(status_of "$result")"
assert_contains "and names what is missing" "$(output_of "$result")" "no executable run"
assert_no_file "and writes no archive" "$archive"

stage_payload "$after"
rm -- "${payload}/cogs/robot_clk_exe"
result=$(pack)
assert_status "a payload without cogs/robot_clk_exe refuses" 1 "$(status_of "$result")"
assert_contains "and names what is missing" "$(output_of "$result")" "no cogs/robot_clk_exe"
assert_no_file "and writes no archive" "$archive"

# Every command the pack runs, except zstd, linked into one directory: the
# refusal has to be the tool check's own sentence, not tar's about a child.
no_zstd="${work}/no-zstd"
mkdir -p -- "$no_zstd"
IFS=: read -ra path_dirs <<<"$PATH"
for d in "${path_dirs[@]}"; do
	if [ -d "$d" ]; then
		cp -ns -- "$d"/* "$no_zstd"/ 2>/dev/null || true
	fi
done
rm -f -- "${no_zstd}/zstd"
stage_payload "$after"
result=$(PATH="$no_zstd" pack)
assert_status "no zstd on PATH refuses" 1 "$(status_of "$result")"
assert_contains "and names the tool" "$(output_of "$result")" "no zstd on PATH"
assert_lacks "in its own words, not tar's" "$(output_of "$result")" "tar:"
assert_no_file "and writes no archive" "$archive"

stage_payload "$before"
result=$(pack)
assert_status "a stale payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says what is wrong" "$(output_of "$result")" \
	"older than the newest commit"
assert_contains "and names the override" "$(output_of "$result")" "--stale-ok"
assert_no_file "and writes no archive" "$archive"
result=$(pack --stale-ok)
assert_status "--stale-ok packs the old payload" 0 "$(status_of "$result")"
assert_contains "and says the age went unchecked" "$(output_of "$result")" \
	"the payload's age is not being checked"
assert_contains "and the archived stamp says so too" "$(archived provenance.txt)" \
	"age_unchecked=yes"

result=$(pack --stale-okay)
assert_status "a misspelled override is a usage refusal" 1 "$(status_of "$result")"

stage_payload "$after"
rm -- "${payload}/build-commit.txt"
result=$(pack)
assert_status "a payload recording no build commit refuses" 1 "$(status_of "$result")"
assert_contains "and says to rebuild" "$(output_of "$result")" "rebuild"

stage_payload "$after"
result=$(BUILD_COMMIT_TIME='' pack)
assert_status "a build commit this checkout does not have refuses" 1 "$(status_of "$result")"
assert_contains "and names it" "$(output_of "$result")" "$BUILD_COMMIT"

# ---------------------------------------------------------------------------
# The archive
# ---------------------------------------------------------------------------

stage_payload "$after"
result=$(pack)
assert_status "a fresh payload packs" 0 "$(status_of "$result")"
assert_file "the archive is where the publish reads it" "$archive"
assert_eq "and it is zstd" "28 b5 2f fd" "$(od -An -tx1 -N4 -- "$archive" | sed 's/^ *//')"
members=$(listing)
for member in ./run ./provenance.txt ./build-commit.txt ./cogs/servo_profile.textproto; do
	assert_contains "the archive root carries ${member}" "$members" "$member"
done
digest=$(sha256sum -- "$archive" | cut -d' ' -f1)
assert_eq "the sidecar is the file's digest, in sha256sum -c shape" \
	"${digest}  payload.tar.zst" "$(cat -- "${archive}.sha256")"
assert_contains "and the report prints it" "$(output_of "$result")" "sha256   ${digest}"

# Modes and ownership: readable by every account after root's unpack, writable
# by nobody but the owner, and the owner root.
modes_ok=yes
owners_ok=yes
while read -r mode owner _; do
	if [ "${mode:5:1}" != - ] || [ "${mode:8:1}" != - ]; then
		modes_ok="no: ${mode}"
	fi
	[ "$owner" = 0/0 ] || owners_ok="no: ${owner}"
done < <(verbose_listing)
assert_eq "no member is writable by group or other" yes "$modes_ok"
assert_eq "every member is owned by 0/0" yes "$owners_ok"
mode_of() { verbose_listing | awk -v m="$1" '$6 == m { print $1 }'; }
assert_eq "run keeps its execute bits" -rwxr-xr-x "$(mode_of ./run)"
assert_eq "a 0600-staged member is readable by the service's account" \
	-rw-r--r-- "$(mode_of ./host/host_params.textproto)"
assert_eq "a configuration file is plain read-only" \
	-rw-r--r-- "$(mode_of ./cogs/servo_profile.textproto)"

# Timestamps and the stamp: the build commit's instant throughout.
want_time=$(date -u -d "@${BUILD_COMMIT_TIME}" '+%Y-%m-%d %H:%M')
times_ok=yes
while read -r _ _ _ day time _; do
	[ "${day} ${time}" = "$want_time" ] || times_ok="no: ${day} ${time}"
done < <(verbose_listing)
assert_eq "every member's time is the build commit's" yes "$times_ok"
stamp=$(archived provenance.txt)
assert_contains "the stamp's instant is the build commit's" "$stamp" \
	"stamped=$(date -u -d "@${BUILD_COMMIT_TIME}" +%Y%m%dT%H%M%SZ)"
assert_contains "and it names the build commit" "$stamp" "commit=${BUILD_COMMIT}"
assert_contains "as the payload's own record" "$stamp" "commit_source=build"
assert_contains "and the packing tree's HEAD beside it" "$stamp" "stamped_from=${GIT_HEAD}"
assert_contains "and the stamp says a pack wrote it" "$stamp" "stamped_by=pack"
assert_contains "and the digest of each run configuration file" "$stamp" \
	"config_sha256=cogs/servo_profile.textproto $(sha256sum -- "${payload}/cogs/servo_profile.textproto" | cut -d' ' -f1)"
assert_contains "and the age was checked" "$stamp" "age_unchecked=no"

# Determinism: the digest is a property of the tree.
first=$(cat -- "${archive}.sha256")
result=$(pack)
assert_status "a second pack of the same tree succeeds" 0 "$(status_of "$result")"
assert_eq "and gives the same archive byte for byte" "$first" "$(cat -- "${archive}.sha256")"
echo '# one byte more' >>"${payload}/cogs/servo_gains.textproto"
result=$(pack)
assert_status "a pack after an edit succeeds" 0 "$(status_of "$result")"
if [ "$first" != "$(cat -- "${archive}.sha256")" ]; then
	pass "and gives a different digest"
else
	fail "and gives a different digest" "both are ${first}"
fi
assert_no_file "no partial archive is left behind" "${archive}.part"

# ---------------------------------------------------------------------------
# Speech configuration
# ---------------------------------------------------------------------------

# A payload staged with both knobs `none` carries neither speech member nor the
# credentials beside them, and the pack adds none of them.
stage_payload "$after"
result=$(pack)
assert_contains "the report says the archive carries no speech configuration" \
	"$(output_of "$result")" "speech configuration  none"
members=$(listing)
for member in ./host/speech.toml ./host/speech-record.toml ./secrets/pod-psk.toml; do
	assert_lacks "an archive of a none-staged payload lists no ${member}" "$members" "$member"
done

printf 'listen_addr = "0.0.0.0:7380"\n' >"${payload}/host/speech.toml"
result=$(pack)
assert_status "a payload carrying a speech configuration still packs" 0 "$(status_of "$result")"
assert_contains "and the report says it is carried" "$(output_of "$result")" \
	"speech configuration  carried"
assert_lacks "and does not tell the operator to build without it" "$(output_of "$result")" \
	"REACHY_SPEECH_CONFIG=none"

# ---------------------------------------------------------------------------

tally
