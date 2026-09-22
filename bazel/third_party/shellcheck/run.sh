#!/usr/bin/env bash
# Adapt bazel run back to the caller's working directory so repository-relative
# file arguments in the Makefile remain valid.
set -euo pipefail

if [[ -z "${RUNFILES_DIR:-}" ]]; then
  if [[ -f "$0.runfiles/MANIFEST" ]]; then
    export RUNFILES_MANIFEST_FILE="$0.runfiles/MANIFEST"
  elif [[ -d "$0.runfiles" ]]; then
    export RUNFILES_DIR="$0.runfiles"
  fi
fi
# shellcheck disable=SC1090,SC1091 # the runfiles library exists only under bazel run
source "${RUNFILES_DIR:-$0.runfiles}/bazel_tools/tools/bash/runfiles/runfiles.bash"

if [[ -z "${BUILD_WORKING_DIRECTORY:-}" ]]; then
  echo "the ShellCheck wrapper is meaningful only under bazel run" >&2
  exit 1
fi
cd -- "$BUILD_WORKING_DIRECTORY"
exec "$(rlocation "$SHELLCHECK_BINARY")" "$@"
