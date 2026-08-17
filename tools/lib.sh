# shellcheck shell=bash
#
# Shared prelude for the tools/ scripts. Sourced, never executed — no shebang,
# not executable:
#
#     # shellcheck source=lib.sh
#     . "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

# Everything here is read by the scripts that source this file, so "appears
# unused" is the expected shape of every definition in it.
# shellcheck disable=SC2034

# The name a script reports itself as in its own messages. Sourcing does not
# change $0, so this is the outer script's path.
prog=$(basename -- "$0")

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# The account the payload runs as on a brenn-os device. A bench run has to
# observe the hardware as this account or it observes nothing worth knowing:
# root opens any device node whatever udev said, so a permission assertion
# taken as root passes vacuously.
app_user=app

# The account's home on the device: writable, mode 0700, and on the volatile
# /var, so it is RAM and costs the eMMC no write. The bench's configuration and
# the state file it writes live here rather than beside the binary — the
# release directory is root-owned and rsync --delete owns its contents.
app_home=/var/lib/brenn-app

# Where brenn-os mounts the payload store. A tmpfs, so nothing under it costs
# the device's flash a write and a reboot clears it; /run itself is noexec and
# this submount deliberately is not, so a binary has to live here to be
# executable at all.
store_mount=/run/brenn-app

# A leading `~` replaced with $HOME, any other path unchanged. Bazel's path
# converter expands a leading `~` and so does the cache action in its path list;
# a shell passing an environment variable through does not, and neither does a
# Go or Rust binary reading one. One implementation, so a path handed to a
# tilde-ignorant tool and a path compared against another copy of itself agree.
#
# The tilde is assembled into a variable rather than written as a literal: the
# linter reads a quoted literal one as a tilde that failed to expand, and it
# would be right about every other use of one.
#
# An unset or empty HOME is a refusal rather than a silent expansion to `/`: the
# callers write the result into an environment variable or compare it against
# another path, and a plausible-looking `/.cache/...` is worse in both places
# than a stop.
expand_home() {
	local tilde='~'
	case $1 in
	"$tilde" | "${tilde}/"*)
		[ -n "${HOME:-}" ] ||
			die "cannot expand a leading tilde in ${1}: HOME is unset or empty"
		;;
	esac
	case $1 in
	"$tilde") printf '%s\n' "$HOME" ;;
	"${tilde}/"*) printf '%s\n' "${HOME}/${1#"${tilde}/"}" ;;
	*) printf '%s\n' "$1" ;;
	esac
}

# Fail with a headline and any number of indented detail lines.
die() {
	echo "${prog}: $1" >&2
	shift
	local line
	for line in "$@"; do
		echo "    ${line}" >&2
	done
	exit 1
}
