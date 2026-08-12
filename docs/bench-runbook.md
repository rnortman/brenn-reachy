# Bench runbook — running `reachy-bench` against a real unit

Everything needed to take this repo from a clean checkout to a measured run on
the hardware, and back again with the numbers. Written because the procedure
used to exist only in Makefiles and shell scripts, and a session spent
reconstructing it is a session not spent measuring anything.

What this does **not** cover: how the device is provisioned, imaged or brought
back after a reboot. That is brenn-pod's
`docs/runbooks/reachy-end-to-end.md`, and it is referenced here rather than
copied.

Before anything that arms, moves or de-torques, the binding rules are in
`docs/fault-management.md`.

## What you need

- **podman**, and on a non-arm64 workstation an `aarch64` `binfmt_misc`
  registration carrying the `F` flag. `tools/build-bench.sh` preflights both
  and says how to fix either.
- **ssh as root to the unit**, key-based: every script runs with
  `BatchMode=yes` and will not prompt.
- **The unit's hostname**, named once in the gitignored `.local/reachy.conf`:

      echo 'REACHY_HOST=reachy00' > .local/reachy.conf

  or per invocation, `make bench-run REACHY_HOST=... ARGS=...`. The variable
  wins over the file, so an override for one invocation stays one invocation.
  Make reads that file; your shell does not, so the `ssh` and loop examples
  below want a `. .local/reachy.conf` first.
- **A configuration for this unit** at `.local/reachy-bench.toml` (override
  with `BENCH_CONFIG=`). Start from
  `crates/reachy-bench/reachy-bench.example.toml`, which is every key at its
  shipped default with the reasoning beside it, and fill in the two things an
  example cannot know: the serial node under `[bus]`, and the `[datum]` table,
  which is commented out and whose absence refuses every command that moves
  something. A person reads a self-test record and writes that table; it is
  not copied from anywhere.

## Where things live

| | |
|---|---|
| `.local/reachy.conf` | the unit's hostname. Gitignored |
| `.local/reachy-bench.toml` | the unit's bench configuration. Gitignored: it holds a serial node and one machine's reviewed datum |
| `.local/records/` | fetched self-test records, and wherever you park retrieved traces |
| `target/bench-arm64/release/reachy-bench` | the device binary `bench-build` produces |
| `crates/reachy-bench/fixtures/traces/` | recorded runs kept as test data |

On the device, and both in RAM — nothing a dev cycle pushes touches the eMMC:

| | |
|---|---|
| `/run/brenn-app/releases/bench/reachy-bench` | the binary. A tmpfs submount, mounted exec where `/run` itself is not. A reboot clears it |
| `/var/lib/brenn-app/` | the `app` account's home, mode 0700: the bench's configuration, the self-test state file, and anywhere you point `--trace` |

Traces and records belong under `/var/lib/brenn-app`. `/run/reachy-trace.csv`
is not writable by the account the run drops to.

## Clearing the bus

Two long-lived processes on the unit open the servo port, and a bench run
wants it to itself:

- `brenn-app.service` — the payload.
- `reachy-motiond.service` — the motion daemon, deployed from brenn-pod.

`deploy-bench.sh` refuses rather than stopping either of them: what is running
on a device is the operator's to decide. Its refusal names the service and the
command, and its exit codes distinguish the two (3 for `brenn-app`, 4 for
`reachy-motiond`) from ssh failing (255) and from the bench's own verdict.

    ssh root@"$REACHY_HOST" systemctl stop reachy-motiond.service
    # ... the session ...
    ssh root@"$REACHY_HOST" systemctl start reachy-motiond.service

**`make reachy-up` in brenn-pod is not a bench command.** It re-pushes the
whole payload and *starts* the motion daemon, which takes the bus back. It is
how a rebooted unit is brought back afterwards, not how a session begins.

## The loop

    make bench-build                     # the aarch64 binary, in the pinned container
    make bench-config                    # push .local/reachy-bench.toml into the unit's RAM
    make bench-run ARGS="selftest"       # read-only: pings and register reads, no torque
    make bench-run ARGS="up"             # and the rest of the commands
    make bench-fetch                     # bring the self-test record back, timestamped

`make bench-run` has `bench-build` as a prerequisite, so the blessed entry
point cannot run a binary older than your tree. `make bench-selftest` is the
same chain for the read-only registry.

`ARGS` is passed to the bench verbatim. Given no command, or one it does not
have, the bench prints its usage, which lists them all. The ones worth knowing
before the first session:

- `selftest` — read-only. Presence, registers, voltage, health, the resting
  pose, the antennas' fold. Writes a state file that `bench-fetch` retrieves.
  A diagnostic and a regression guard; it gates nothing.
- `arm` / `off` — take hold, and let go. `off` always releases: wherever the
  machine is, torque comes off and where it was is reported. That is the way
  out of any session at any moment, and the head settles as it goes, so take
  its weight if it is up.
- `up`, `stow`, `hold`, `yaw <deg>`, `antennas <right> <left>`, `demo`.
  Every one of them commissions the machine and takes hold of it first —
  nothing is remembered between invocations — and every one but `demo` leaves
  the machine holding when it ends, so `up`, `yaw`, `stow` chain without the
  head dropping in between. Finish with `off`.
- `reboot [id]` — see below.

A clean move prints its period count, jitter, slip and elapsed time; the two
instants it ended on (commanding finished, and measurably at the goal); and
the worst lag every joint ran at. A run that faults prints the fault, the
maneuver that answered it, and the session's whole record as one `incident:`
line.

## Binary freshness

`deploy-bench.sh --run` refuses a binary older than the newest commit touching
`crates`, `containers`, `Cargo.toml`, `Cargo.lock` or `rust-toolchain.toml`.
The refusal prints both times and names `make bench-build`, and that build
always clears it: `build-bench.sh` stamps the artifact once it has verified it,
so a commit cargo had nothing to relink for — a checked-in trace fixture, a
test-only change — does not leave a current binary looking stale.

This exists because a day-old binary once resurrected a config gate that had
been deleted from the source, and cost an evening. Two things it does not
catch, both deliberate:

- **Uncommitted edits.** Commit time against file time cannot see them.
  `make bench-run` builds first, which is the answer to that half, and is the
  entry point to prefer for exactly this reason.
- **A tree with no history for those paths** (a tarball, a clone that excluded
  them). It says the age is unknown and runs. Absence of history is not
  evidence of staleness.

To run an old binary deliberately — bisecting a behaviour, reproducing a past
session — `--stale-ok` is the first token after `--run`:

    tools/deploy-bench.sh "$REACHY_HOST" --run --stale-ok up

It is this script's flag, not the bench's, so it must be in that position;
everything from the next token on goes to the bench untouched.

## `reboot` versus a power cycle

A latched hardware error — an antenna overload above all — makes a servo
refuse to be enabled until the latch clears. `reboot` sends the servos'
restart instruction and ends the walk to the power switch.

    make bench-run ARGS="reboot"       # all nine
    make bench-run ARGS="reboot 17"    # one, by bus ID

It gates on nothing, and it drops torque: whatever a servo was holding, it
lets go of. **Take the head's weight if it is up.** The command waits for each
servo to answer again and refuses to call the restart done on a servo that
comes back still holding torque, or one that never acknowledged and came back
limp — either is an unconfirmed restart, not a success.

What a restart does, and how it compares with cutting power:

| | `reboot` | power cycle |
|---|---|---|
| latched hardware error | cleared | cleared |
| torque | dropped | dropped |
| multi-turn count on the antennas | folds into one turn | folds into one turn |
| health byte immediately after | `0x00` observed | the chronic `0x01` observed |
| how long | seconds, from the workstation | a walk to the switch |

Arming writes the gains and the motion profile at every arm, so neither path
can leave a session running on stale ones.

The antennas turn freely and are in extended-position mode, so their reported
angle can accumulate past a turn while the machine runs. Both paths fold that
count back into a single turn; the self-test's `antenna-fold` case is what
asserts it, and a reading outside one turn is a finding for a person, never a
bound to widen. See the open observations at the end.

## Traces, and what to do with one

`--trace PATH` writes one CSV row per control period: every joint's measured
angle against the goal it was being held to, which is the move's velocity
profile at the rate it was sampled. Each run of the invocation appends, and
the run number is derived from the file's own last row, so several
invocations against one path accumulate rather than overwrite.

    make bench-run ARGS="up --trace /var/lib/brenn-app/reachy-trace.csv"
    ssh root@"$REACHY_HOST" cat /var/lib/brenn-app/reachy-trace.csv \
        > .local/records/trace-$(date -u +%Y%m%dT%H%M%SZ).csv

A released joint's goal cells are blank from the period after it went out of
service; its measured cells keep recording. That is the diagnostic of record
for a degraded run.

**Reading one back.** `reachy_bench::trace::metrics` is the measurement half
of the old scratch analyzer, in Rust and under test: `Trace::read` parses a
file into runs and periods, and the per-run and per-joint measurements are
span, peak measured and goal speed, peak goal step, worst lag, arrival,
residual, settle gap, longest stall, and the antenna pair's phase separation.
There is no CLI over it — its consumers are the fixture tests below. The
plotting half stays scratch in `.local/`, with its own throwaway venv; a
plotting aid is bench-session tooling, not product.

Two cautions the module states at its source and that matter when you read a
number off an old file. A goal column is what the loop *commanded*, and
recordings made before the per-period move clock carry whatever the scheduler
did that night inflated into their step sizes — no guard is ever sized against
a recorded step. A measured column is the servo's own encoder at the rate the
loop read it, so velocities from it are averages over a period.

**Keeping one.** A recording that settled a question becomes a fixture:
`crates/reachy-bench/fixtures/traces/`, a row in that directory's `README.md`
saying what it records, and a test in `trace::metrics` asserting the property
that qualified it. The four already there are what the shipped step bounds,
tracking threshold, gains and antenna clocks are sized against — a measurement
nothing replays is folklore by the next release. `*.csv` is gitignored
repo-wide and that directory is negated back in, so a new fixture lands with
`git add` like anything else.

## The 50-cycle soak

The acceptance gate before the machine goes back to unattended service, and
the empirical check on the antenna phase-separation margin. No bench command
exists for it; it is a loop over the ordinary gesture commands with a trace.

Preconditions: the unit's configuration rebuilt from the current example (the
soak runs at **shipped defaults**, not at a trial configuration), motiond and
`brenn-app` stopped, the machine limp and stowed, a fresh binary, and nothing
on or around the head.

    . .local/reachy.conf
    make bench-build
    trace=/var/lib/brenn-app/soak.csv
    for i in $(seq 50); do
        tools/deploy-bench.sh "$REACHY_HOST" --run up   --trace "$trace" || break
        tools/deploy-bench.sh "$REACHY_HOST" --run stow --trace "$trace" || break
    done 2>&1 | tee .local/records/soak-$(date -u +%Y%m%dT%H%M%SZ).log
    tools/deploy-bench.sh "$REACHY_HOST" --run off

It passes on all four of:

1. **Zero faults.** No run prints a fault or an `incident:` line; the loop
   never breaks.
2. **Zero latched error bits afterwards.** `make bench-selftest` then
   `make bench-fetch`, and the `health` case passes — nothing beyond the
   chronic input-voltage bit noted below.
3. **Settle within tolerance every run.** Every move printed the arrival form
   of its settle line ("measurably at the goal ... later"), never the form
   naming a joint that was still out when the window ran out.
4. **The ears check.** Listen at the head while it holds between cycles. The
   antennas ship with a derivative term they have not been listened to at
   length with; an audible buzz at hold is a finding, and the answer is a
   smaller antenna D gain and another soak.

Retrieve `soak.csv` afterwards as above — 100 runs in one file — and keep it
if anything in it is worth arguing about later.

A tip contact during the soak is that margin's verdict, and the margin is
configuration: `antenna_phase_separation_rad` in `[motion]`, with
`antenna_contact_band_rad` beside it. Try a wider figure on the bench, re-run
the soak, and propose the value with the recording that argues for it —
promoted to a fixture, it is what the replay tests re-derive against.

## Open observations

Anomalies with no explanation yet. They live here rather than in a chat log so
that the next person to see one knows it is not new.

- **A 545° antenna reading immediately after a hard power cycle.** Seen once.
  A power cycle is supposed to fold the multi-turn count into a single turn,
  and two other power-on observations plus every observed `reboot` do exactly
  that. Nothing explains the one that did not. The self-test's `antenna-fold`
  case is the tripwire: it reads each antenna's count from the resting sweep
  and fails by name outside one turn. If it ever fails, that is this
  observation recurring — take it to a person and do not widen the bound.
- **The chronic `0x01` input-voltage latch.** All nine servos latch the
  input-voltage bit during ordinary running. A `reboot` clears them to `0x00`
  and running re-latches them. The suspicion is a supply dip under load, and
  the measurement that would settle it is the rail under the current nine
  servos draw taking up the head's weight. The bit is deliberately excluded
  from the health predicate — the engage-time supply gate is what owns supply
  — so this does not stop anything today; it is an unexplained reading on a
  machine whose other readings are trusted.
