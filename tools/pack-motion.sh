#!/usr/bin/env bash
#
# Pack the staged motion payload as the archive a device installs.
#
#   tools/pack-motion.sh [--stale-ok]
#
# Reads target/motion-arm64/release/, the payload `build-motion.sh` staged,
# writes the provenance stamp into it with the push's own function, and packs it
# as target/motion-arm64/payload.tar.zst with its sha256 beside it in
# payload.tar.zst.sha256. The archive is what the boot fetch and a resync
# install and what a later bake takes: the payload's contents at the archive
# root, `run` among them, not inside a directory.
#
# Every flag that could vary between two packs of one tree is pinned: members in
# name order, every timestamp the build commit's, owner and group 0, and modes
# `u=rwX,go=rX`, which is what makes every member readable by the account that
# runs the payload after root unpacks it and writable by nobody. So two packs of
# one staged tree give one archive byte for byte: a bake's digest is a property
# of what was staged, not of when it was packed. The staged tree is not a
# function of the commit alone -- it carries site files (the speech
# configuration and what it names, the host parameters, the wake models) and the
# pod binary -- so republishing an earlier build means keeping its archive, not
# rebuilding it.
#
# Refuses a payload older than the newest commit to what it is built from, the
# way `deploy-motion.sh --push` does; `--stale-ok` packs it anyway and the stamp
# says so.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

payload="${repo_root}/target/motion-arm64/release"
archive="${repo_root}/target/motion-arm64/payload.tar.zst"
digest_file="${archive}.sha256"

usage() {
	die "usage: ${prog} [--stale-ok]"
}

age_unchecked=no
if [ $# -eq 1 ] && [ "$1" = --stale-ok ]; then
	age_unchecked=yes
elif [ $# -ne 0 ]; then
	usage
fi

# `tar --zstd` execs `zstd`, so a missing one is a tar error about a child
# process rather than a sentence about what to install.
for tool in tar zstd; do
	command -v "$tool" >/dev/null 2>&1 ||
		die "there is no ${tool} on PATH, and the archive is a zstd-compressed tar."
done

[ -d "$payload" ] ||
	die "no device payload at ${payload}; build one first: make motion-build"
[ -x "${payload}/run" ] ||
	die "the payload has no executable run at its root; rebuild it: make motion-build"
[ -f "${payload}/cogs/robot_clk_exe" ] ||
	die "the payload has no cogs/robot_clk_exe; rebuild it: make motion-build"

if [ "$age_unchecked" = yes ]; then
	echo "${prog}: --stale-ok: the payload's age is not being checked" >&2
else
	refuse_if_stale "${payload}/cogs/robot_clk_exe" "device payload" \
		"make motion-build" \
		"pack the old payload deliberately" \
		"${prog} --stale-ok" \
		"${workspace_paths[@]}"
fi

# The build commit's own instant is every member's timestamp and the stamp's
# `stamped=`: the one time a payload has that depends on the tree alone.
built=$(sed -n 's/^commit=//p' -- "${payload}/${build_commit_name}" 2>/dev/null) || built=
case $built in
'' | unknown)
	die "the payload records no build commit, so the archive's timestamps have nothing deterministic to be set to; rebuild it: make motion-build"
	;;
esac
epoch=$(git -C "$repo_root" show -s --format=%ct "$built" 2>/dev/null) || epoch=
[ -n "$epoch" ] ||
	die "this checkout has no commit ${built}, which the payload says it was built from; fetch it or rebuild: make motion-build"

# The stamp reads these from its caller: a pack lays no overlay, and the wake
# models are the ones the staged configuration names.
experiment_dir=
staged_wake_models=$(speech_model_paths "${payload}/${speech_config_path}") || exit 1
stamp_provenance "${payload}/${provenance_name}" "$age_unchecked" pack \
	"$(date -u -d "@${epoch}" +%Y%m%dT%H%M%SZ)"

# Written beside the archive and renamed into place, so an interrupted pack
# leaves the previous archive whole rather than a prefix under its name.
part="${archive}.part"
trap 'rm -f -- "$part"' EXIT
rm -f -- "$part"
tar --zstd --sort=name --mtime="@${epoch}" --owner=0 --group=0 --numeric-owner \
	--mode='u=rwX,go=rX' -cf "$part" -C "$payload" .
mv -- "$part" "$archive"

digest=$(sha256_of "$archive")
printf '%s  %s\n' "$digest" "$(basename -- "$archive")" >"$digest_file"

echo "${prog}: archive  ${archive}  ($(du -h -- "$archive" | cut -f1))"
echo "${prog}: sha256   ${digest}"
if [ -f "${payload}/${speech_config_path}" ] || [ -f "${payload}/${record_speech_config_path}" ]; then
	echo "${prog}: speech configuration  carried"
else
	echo "${prog}: speech configuration  none"
fi
