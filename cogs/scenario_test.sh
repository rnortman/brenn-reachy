#!/usr/bin/env bash
# One scenario, end to end: author the input log, drive the system from it under
# the deterministic runner, and check the output log.
#
# Every scenario of every system in this repo runs through this script, so the
# runner is invoked one way. A scenario whose invocation differed from the others
# would be asserting about a different run than it claims to, and the difference
# would be one flag in one copy of eighty lines.
#
# Deterministic, so these are tests and not demos: a run is bounded by simulated
# time, the end time comes from the scenario rather than from a stopwatch, and
# the number of executions does not depend on this machine.
#
# Hermetic: both logs and the pinion channel buffers go under the test's own
# temporary directory, in a namespace of its own, so nothing touches machine-wide
# shared memory and two runs cannot see each other.
#
# $1 the pinion namespace, $2 the author binary, $3 the process executable, $4
# the process description, $5 the channel publisher config, $6 the event log
# writer config, $7 the checker binary, and $8 onwards whatever else that
# checker takes -- all passed by the test target as runfiles-relative paths. The
# simulated end time is not an argument: the author prints it, so a scenario's
# schedule stays stated in one place.
set -euo pipefail

pinion_ns="$1"
author="$PWD/$2"
exe="$PWD/$3"
process_description="$PWD/$4"
channel_publisher_config="$PWD/$5"
log_writer_config="$PWD/$6"
checker="$PWD/$7"
shift 7
checker_args=()
for arg in "$@"; do
    checker_args+=("$PWD/$arg")
done

input_log="$TEST_TMPDIR/input.slog.d"
output_log="$TEST_TMPDIR/output.slog.d"
run_log="$TEST_TMPDIR/run.out"

fail() {
    echo "$pinion_ns: $*" >&2
    exit 1
}

sim_end_ns="$("$author" "$input_log")" || fail "the author refused to write the input log"
[[ "$sim_end_ns" =~ ^[0-9]+$ ]] || fail "the author named no simulated end time"

# The runner classifies a log by what is in the directory, so an input log with
# no `.slog` file in it would be reported as an unknown format rather than as an
# empty scenario. Checking here separates "the writer wrote nothing" from "the
# runner would not read it".
shopt -s nullglob
input_files=("$input_log"/*.slog)
shopt -u nullglob
[[ ${#input_files[@]} -gt 0 ]] || fail "the author wrote no .slog file under $input_log"

# The start time is left to default from the input log's own first message; only
# the end is stated, because the scenario's tail is a choice and the start is a
# fact about the log. The status comes off the command itself: inside `if ! cmd`
# the shell reports the negation's status, which is always zero.
status=0
"$exe" "$process_description" \
    --deterministic-runner \
    --sim-end-time-ns "$sim_end_ns" \
    --input-log-uri "$input_log" \
    --channel-publisher-config "$channel_publisher_config" \
    --output-log-uri "$output_log" \
    --log-writer-config "$log_writer_config" \
    --pinion-dir "$TEST_TMPDIR" \
    --pinion-ns "$pinion_ns" \
    >"$run_log" 2>&1 || status=$?

if [[ $status -ne 0 ]]; then
    cat "$run_log" >&2
    fail "the deterministic run exited $status"
fi

# Expanded through the set-if-set form: under `set -u` an empty array is an
# unbound variable on bash before 4.4, and a scenario whose checker takes no
# extra arguments has exactly that. The failure would read as a harness bug on
# whichever machine ships the older shell.
"$checker" "$output_log" ${checker_args[@]+"${checker_args[@]}"} || {
    cat "$run_log" >&2
    fail "the output log did not match the scenario"
}
