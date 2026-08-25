# Bench runbook — running this repo's binaries against a real unit

Everything needed to take this repo from a clean checkout to a measured run on
the hardware, and back again with the numbers. Written because the procedure
used to exist only in Makefiles and shell scripts, and a session spent
reconstructing it is a session not spent measuring anything.

Two runs live here, and they are different things. `reachy-bench` validates that
we can talk to the hardware — reads, provisioning, releasing torque; it moves
nothing. The motion test runs the cog system and its driver, and it is the run
that moves the machine. Everything up to **The motion test** below is the bench;
that section and `Where things live` cover both.

What this does **not** cover: how the device is provisioned, imaged or brought
back after a reboot. That is brenn-pod's
`docs/runbooks/reachy-end-to-end.md`, and it is referenced here rather than
copied.

Before anything that arms, moves or de-torques, the binding rules are in
`docs/fault-management.md`.

## What you need

- **bazel** (bazelisk; `.bazelversion` pins the release it fetches).
  `tools/build-bench.sh` preflights it and says how to get it. The device binary
  is a cross-compile against the hermetic sysroot the pinned Clockwork drop
  brings — no container, no emulation. It needs an x86_64 workstation: the drop's
  toolchains only run there, so an arm64 development host cannot build the device
  binary today.
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
  which is commented out. A person reads a self-test record and writes that
  table; it is not copied from anywhere. Nothing in that file configures
  motion — the bench moves nothing.

## Where things live

| | |
|---|---|
| `.local/reachy.conf` | the unit's hostname. Gitignored |
| `.local/reachy-bench.toml` | the unit's bench configuration. Gitignored: it holds a serial node and one machine's reviewed datum |
| `.local/records/` | fetched self-test records, and wherever you park retrieved traces |
| `target/bench-arm64/release/reachy-bench` | the device binary `bench-build` produces |
| `target/motion-arm64/release/` | the motion payload `motion-build` stages: both binaries and every configuration file the three processes read, laid out the way their relative paths expect |
| `.local/motion-logs/` | fetched run logs, one directory per fetch |
| `crates/reachy-motion/fixtures/traces/` | recorded runs kept as test data |

On the device, and all in RAM — nothing a dev cycle pushes touches the eMMC:

| | |
|---|---|
| `/run/brenn-app/releases/bench/reachy-bench` | the bench binary. A tmpfs submount, mounted exec where `/run` itself is not. A reboot clears it |
| `/run/brenn-app/releases/motion/` | the motion payload, and the working directory its three processes are started from |
| `/run/brenn-app/logs/motion/` | where the logger writes a run's `.olog` files, one directory per run |
| `/var/lib/brenn-app/` | the `app` account's home, mode 0700: the bench's configuration and the self-test state file |

Records belong under `/var/lib/brenn-app`: it is the one directory the account
the run drops to can write.

## Clearing the bus

Two long-lived processes on the unit open the servo port, and a bench run
wants it to itself:

- `brenn-app.service` — the payload.
- `reachy-motiond.service` — the motion daemon, deployed from brenn-pod.

`deploy-bench.sh` refuses rather than stopping either of them: what is running
on a device is the operator's to decide. Its refusal names the service and the
command, and its exit codes distinguish the two (3 for `brenn-app`, 4 for
`reachy-motiond`) from ssh failing (255) and from the bench's own verdict.
`deploy-motion.sh --push` asks the same question before it pushes anything — a
motion run wants the bus as much as a bench run does — and it is literally the
same question: the probe, the codes and the refusals live in `tools/lib.sh`, so
the contract described here is one thing rather than two that agree today. For
the bench that question gates the run, because the probe and the run share one
ssh invocation; for the motion deploy it is advisory, because the push is a
second connection and the run is started by hand afterwards.

    ssh root@"$REACHY_HOST" systemctl stop reachy-motiond.service
    # ... the session ...
    ssh root@"$REACHY_HOST" systemctl start reachy-motiond.service

**`make reachy-up` in brenn-pod is not a bench command.** It re-pushes the
whole payload and *starts* the motion daemon, which takes the bus back. It is
how a rebooted unit is brought back afterwards, not how a session begins.

## The loop

    make bench-build                     # the aarch64 binary, cross-compiled by bazel
    make bench-config                    # push .local/reachy-bench.toml into the unit's RAM
    make bench-run ARGS="selftest"       # read-only: pings and register reads, no torque
    make bench-run ARGS="off"            # and the rest of the commands
    make bench-fetch                     # bring the self-test record back, timestamped

`make bench-run` has `bench-build` as a prerequisite, so the blessed entry
point cannot run a binary older than your tree. `make bench-selftest` is the
same chain for the read-only registry.

`ARGS` is passed to the bench verbatim. Given no command, or one it does not
have, the bench prints its usage, which lists them all. The ones worth knowing
before the first session:

- `selftest` — read-only. Presence, registers, the legs' travel fences, voltage,
  temperature, health, the resting pose, the antennas' fold. Writes a state file
  that `bench-fetch` retrieves. The temperature case has never been run against a
  unit: the register is untried here, so a reading outside `5..=55 °C` is a
  person's to review before the band moves. The `leg-fence` case reads each leg
  servo's own travel window back and compares it against the motion envelope the
  cog system commands through — a disagreement in either direction names the leg
  and prints both windows in counts and degrees, and is a mis-provisioned unit
  rather than something to widen the tolerance for.
  A diagnostic and a regression guard; it gates nothing.
  **Power-cycle the unit before a sweep**, or run it before anything else in a
  session: the Bus Watchdog register is RAM-resident and resets to 0 at
  power-on, and the sweep checks it against that provisioned 0. A session — or
  the `watchdog` command below — leaves 10 in it, and a sweep taken afterwards
  fails on that register and nothing else. That failure is exact and benign, and
  the check stays hard rather than tolerating stale session state.
- `provision` — writes the antennas' operating mode. Torque must be off on
  both; it moves nothing.
- `off` — writes torque off on every servo on the roster. It gates on nothing
  and measures nothing: wherever the machine is, torque comes off. Every servo
  is asked whatever the ones before it answered, and a servo that never
  acknowledged fails the command rather than riding out on the closing line.
  The head settles as it goes, so take its weight if it is up.
- `reboot [id]` — see below.
- `watchdog [id]` — see below. The one command here that holds torque.

**There is no command here that moves the machine, and there will not be
one.** This tool validates that we can talk to the hardware: the read-only
registry, provisioning, `reboot`, `off`, and the `watchdog` assertion — so the
machine can always be read and always be released. `watchdog` is the one that
holds torque, and it commands no angle either: the only goal it ever writes is
the count the servo reports for itself. The supervised motion commands — `arm`, `up`, `hold`,
`stow`, `yaw`, `antennas`, `demo`, `play` — and everything under them, the
fixed-rate pump and the trace writer included, are deleted rather than parked.
Coordinated motion is the cog system's, and a motion test on a unit is a run of
that system, not of this binary.

## Binary freshness

`deploy-bench.sh --run` refuses a binary older than the newest commit touching
`crates`, `bazel`, `MODULE.bazel`, `MODULE.bazel.lock`, `.bazelrc`,
`.bazelversion` or `tools/build-bench.sh` — the last of those because the
platform and compilation mode the device build uses live only there. The refusal
prints both times and names `make bench-build`, and
that build always clears it: `build-bench.sh` installs the verified output at the
contract path, which stamps it, so a commit the binary did not have to relink for
— a checked-in trace fixture, a test-only change — does not leave a current
binary looking stale.

This exists because a day-old binary once resurrected a config gate that had
been deleted from the source, and cost an evening. Three things it does not
catch, all deliberate:

- **Uncommitted edits.** Commit time against file time cannot see them.
  `make bench-run` builds first, which is the answer to that half, and is the
  entry point to prefer for exactly this reason.
- **A tree with no history for those paths** (a tarball, a clone that excluded
  them). It says the age is unknown and runs. Absence of history is not
  evidence of staleness.
- **`.bazelrc.user`.** It is untracked by design and can override any build
  flag, so no commit time describes it. A per-developer flag that changes what
  the device binary is has to be followed by a rebuild by hand.

To run an old binary deliberately — bisecting a behaviour, reproducing a past
session — `--stale-ok` is the first token after `--run`:

    tools/deploy-bench.sh "$REACHY_HOST" --run --stale-ok selftest

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

It also judges the byte it went in for. A servo that answers its restart still
carrying any hardware-error bit fails the command: this machine clears the byte
to `0x00` on a restart, so a bit that survives means either the restart did not
take or the condition is live at that instant, and a recovery command that did
not recover says so in its exit code. No bit is carved out, the chronic
input-voltage `0x01` included — the closing line names the servo and its byte,
and the exit is non-zero. Every servo's byte is printed either way, so a run that
passes is still the reading of record.

What a restart does, and how it compares with cutting power:

| | `reboot` | power cycle |
|---|---|---|
| latched hardware error | cleared | cleared |
| torque | dropped | dropped |
| multi-turn count on the antennas | folds into one turn | folds into one turn |
| health byte immediately after | `0x00` observed, and asserted: a bit that survives fails the command | the chronic `0x01` observed |
| how long | seconds, from the workstation | a walk to the switch |

Whatever arms this machine writes the gains and the motion profile at every
arm, so neither path can leave a session running on stale ones.

The antennas turn freely and are in extended-position mode, so their reported
angle can accumulate past a turn while the machine runs. Both paths fold that
count back into a single turn; the self-test's `antenna-fold` case is what
asserts it, and a reading outside one turn is a finding for a person, never a
bound to widen. See the open observations at the end.

## The `watchdog` self-test

The servos' own Bus Watchdog is what answers a driver that was killed, crashed
or unplugged with the machine under torque: the register counts bus silence in
20 ms units, and a session arms it at 10 — 200 ms — on every servo, every
engagement. Two things about it are vendor documentation nobody here has watched
happen, and the whole de-energize story rests on both: that ordinary bus traffic
resets the count, and that a servo whose count runs out **stops holding**.

    make bench-run ARGS="watchdog"        # an antenna, the default
    make bench-run ARGS="watchdog 17"     # a servo you name

Supervised, at rest, on one servo — an antenna unless you name another, because
the servo is going to let go and an antenna letting go costs the least. **Never
with the head up.** It runs for a few seconds and asserts, in order:

1. arming at 10 reads back 10,
2. a released servo reads its goal register as its present position, which is
   what makes torquing it a hold rather than a move,
3. five timeouts of reads and goal rewrites at 20 ms do not trip it,
4. five timeouts of reads alone do not trip it either,
5. two timeouts of true silence do: the register reads `0xff`, a goal rewrite
   comes back refused with Data Range, and the servo is no longer holding,
6. a zero clears the trip, re-arming takes, and torque can be enabled again,
7. it leaves that servo disarmed and limp.

Torque only ever goes on with the servo's own present position written as its
goal first, so neither the initial hold nor the re-arm after the trip can be a
commanded move — the servo has been limp in between and may have drooped.

Every one of those is an assertion, so a failure is the discovery and the run's
exit code says so. **An unexpected reading goes to a person before anything is
changed to make it pass** — assertion 4 is the one to expect trouble from, and
the recorded contingency if reads turn out not to feed the watchdog is a
driver-side keep-alive write while torqued. Do not implement it before the
reading says it is needed.

Run this **before the first motion run**, and power-cycle afterwards if a
read-only sweep comes next: this command leaves the register at 0 on the servo it
addressed, but a session leaves 10 on all nine.

## The motion test

The run that moves the machine. Not a bench command and never will be: motion
lives in the cog system, so a motion test on a unit is a run of that system.

**Nothing here has been run on hardware yet.** Every Reachy ADR since
2026-08-14 records no unit available, so what follows is the procedure and the
gates, written before the first run rather than after it. The first person to do
it is discovering things; the bring-up rule applies — an unexpected reading goes
to a person before anything is changed to accommodate it.

**Before the first motion run on a unit, run the `watchdog` self-test** (above)
and then power-cycle. The de-energize story this section tells has two layers,
and that test is what establishes the lower one on this machine: until it has
passed here, the 200 ms watchdog is armed by every session and its semantics are
unverified, which means a killed or crashed driver may leave the machine holding.
The launcher's own stop gesture does not depend on it.

### Before you take it to a unit: the same system on your workstation

    make motion-host-run

This starts the whole online system here — the real control process, the real
logger, and the simulated plant in a process of its own behind the same six
loopback ports `reachy_motord` binds — under the same launcher, from the same
kind of rendered config, on the same shared-memory defaults. It runs the wake
gesture, stops, and judges the log with `first_motion_report`, and the target's
verdict is the report's.

Run it before a bench session and after any change to the compositions or the
configs. What it catches is everything about the *configuration* that a unit
would otherwise be the first to find: a process description nothing can find, a
pinion namespace that disagrees, a launcher config naming a path that is not
there, a writer that cannot open its directory, a seam schema whose two ends
disagree about a datagram's size, a log the analyzer cannot read.

What it cannot tell you: nothing about serial timing, nothing about direct I/O on
*this unit's* kernel, nothing about aarch64, and nothing whatever about real
servos — which is also why the device-config-only smoke test this replaces is not
worth building. The two configuration facts it does not cover are the driver's
serial device and the device log root, and those are precisely the two a
workstation cannot check.

The plant is a cog woken by the wall-clock runner, so it stamps its samples with
when it ran rather than with an exact grid instant; the harness passes the
analyzer a jitter band for that reason and says so. A hardware log is read with
no band, because a driver computes its instants.

### What runs

Three OS processes, two binaries, one supervisor:

| | |
|---|---|
| `simplelaunch` | the launcher, from the pinned Clockwork drop. Starts all three processes from `robotcpu.textproto`, redirects each one's console into its own file, and stops them all together |
| `reachy_motord` | the servo bus. Not a cog: it owns the serial port, a 20 ms grid on the real clock, and the driver decisions. Talks to the cogs over six UDP sockets on loopback |
| `cogs/robot_clk_exe` + the logger process description | writes the run's `.olog` records. Observation only — no channel leaves it into the control loop, so a dead logger costs records and nothing else |
| `cogs/robot_clk_exe` + the control process description | the control loop: `Mover`, `Pose`, `Session`, and the wake gesture that asks for the one thing this system does unprompted |

One executable, two processes: which one it becomes is the process description
it is started with. `robotcpu.textproto` is not written by hand — the
compositions render it from `cogs/robot.clk`, and the payload carries it as
rendered. It names two of the apps itself; the driver is merged into it from
`driver/motord_launch.textproto`, because the renderer only knows about
Clockwork processes.

Nothing carries a `--pinion-dir` or a `--pinion-ns`. The rendered config cannot
express per-process flags, so every process runs on the compiled-in defaults —
`/dev/shm`, and the empty namespace, which is a namespace like any other. The
logger is handed those two values as configuration rather than on a command
line, so `cogs/robot_logger.textproto` restates them, and `make motion-build`
refuses to stage a payload where it says anything else. That refusal is the whole
of the namespace agreement: there is no longer a start command for it to drift
from.

### The loop

    make motion-build       # the aarch64 payload: the launcher, both binaries, every config
    make motion-deploy      # push it into the unit's RAM, and make the log directory
    make motion-commands    # print the start command, the logs to watch, and the stop
    # ... the run ...
    make motion-fetch       # bring the .olog directories back, timestamped
    bazel run //cogs:first_motion_report -- .local/motion-logs/<fetch>/<run>

`motion-deploy` has `motion-build` as a prerequisite, so the blessed entry point
cannot push a payload older than your tree. It refuses while `brenn-app.service`
or `reachy-motiond.service` is running — either of them holds the servo bus —
and the refusal names the service and the command, exactly as the bench's does.
That question is advisory here, which it is not for the bench: a push starts
nothing, and the run itself is a command a person types later. What keeps two
claimants off the bus at that point is the driver's exclusive open of the port,
so clear the bus before you start the run and not merely before you push.

The payload lands at `/run/brenn-app/releases/motion` — tmpfs, cleared by a
reboot, and the working directory the launcher is started from, because every
path in its config, and every configuration file each process reads, is named
relative to it. The records land under `/run/brenn-app/logs/motion` and the
processes' console output under `/run/brenn-app/logs/launch`, also RAM: pull
what you want off with the run.

### Starting, watching, stopping

`make motion-commands` prints them. One command starts the run:

    cd /run/brenn-app/releases/motion && ./simplelaunch robotcpu.textproto \
        --logdir /run/brenn-app/logs/launch

It runs in the foreground. Watch the driver with `tail -f` on
`/run/brenn-app/logs/launch/motord_0.log`, and the session on `proc_0.log`
beside it; a second run in the same log directory writes `motord_1.log`, and so
on.

**There is no start order to get right, and none to get wrong.** The launcher
spawns all three at once, in whatever order it happens to walk them, and each is
safe alone: a driver with nobody talking to it reaches the minimum risk
condition inside its startup window and says so; a control loop whose driver is
not up yet finds the servos absent — its first presence ping has an 800 ms
delivery budget against a port bound milliseconds after the fork — and parks the
session without ever writing torque, which is a dead run and never a hazard; a
logger that starts late costs early records and nothing else.

**Stopping is one gesture, and any way of stopping is safe.** Ctrl-C in the
launcher's terminal, or `curl -X POST 127.0.0.1:8080/quit` from anywhere on the
unit. The launcher SIGINTs all three children: the cog processes stop cleanly,
and the driver de-torques the machine, confirms it, prints what it did, and
exits — well inside the 3 s before the launcher escalates to SIGTERM. What
covers everything a signal cannot — SIGKILL, a crash, a yanked cable, a wedged
host — is the servos' own Bus Watchdog, armed by the session at 200 ms: a head
held by nobody goes limp on its own. That claim about the watchdog is asserted by
the bench `watchdog` self-test and is **unverified on hardware** until that test
has passed on this unit; run it before the first motion run.

If the launcher refuses to start with a complaint about a running process, that
is its own check: it will not start an app whose executable basename is already
running. A stray `robot_clk_exe` or `reachy_motord` from an earlier experiment is
what that means. Find it and stop it; the refusal is the port-bind refusal you
would otherwise have hit later and less clearly.

The launcher also listens on 127.0.0.1:8080 for the duration of the run. That is
one more local-trust-only surface on the loopback seam this machine already has,
and no new class of exposure.

### What a good run looks like

The wake gesture, once, and then nothing: commission, engage, raise, hold, stow,
verified release, `resting`. The machine ends de-torqued and the session ends
rest-class, which is not a fault — a rest-class ending is the machine at the
minimum risk condition, and the next wake is a fresh session rather than a
recovery. The gesture is `cogs/wake_params.textproto`; it is the same gesture
the S1 scenario runs, so every phase of it is already pinned by a test.

Expect the first dozen seconds to look like nothing happening. Commissioning is
about five seconds of bus transactions with the machine still, and the wake lead
— `lead_ms` in `cogs/wake_params.textproto`, eight seconds — is deliberate room
for that survey to finish before the first step opens. A motionless machine in
that window is the normal shape of a good run, not a wedged one; the survey is
visible meanwhile in the driver's log, which is what the `tail` above is for.

Watch for it with your eyes on the machine and your verdict from the log.
`first_motion_report` over the fetched records is the verdict: the phase
sequence in order, zero tick faults, no `bus_failure` and no `cycle_skipped`, a
release with evidence, and the measured half printed either way — per-cycle
jitter, read losses, tracking lags beside the replay suite's pinned headroom,
the health rotation's readings, and a per-channel census. It prints the
measurements whether it passes or fails, because a first run that fails is
exactly the run whose numbers somebody needs. Its own test runs it over the S1
scenario's deterministic log, so the analyzer was proved before any hardware log
existed.

**Before you power the unit down, look at the log root while it is still up:**

    ls -l /run/brenn-app/logs/motion/*/

A run directory holding a non-zero-length `.olog` is the only evidence that the
writer got as far as records; that directory is tmpfs and a power-down takes the
answer with it. `make motion-fetch` refuses a fetch that brought no records
rather than printing a report command over nothing, but by then the machine may
be off and the run unrepeatable — so the observation belongs here, while it is
still a question you can act on.

File the report and the log with the run record. Four things to check on the unit
the first time, none of which any test here can answer: that the log directory's
filesystem accepts direct I/O on that kernel — the writer opens every file
`O_DIRECT | O_DSYNC` and a refusal is a failed open at the first file — that the
aarch64 binaries run at all, which `make check-device` says nothing about because
it builds and never executes them, that the arrival tolerances the report judges
by are the ones you want — `ARRIVAL_OFFSET_M`, `ARRIVAL_TURN_RAD` and
`ARRIVAL_ANTENNA_RAD` in `cogs/first_motion_report.rs` were sized from the
mechanism on paper by an agent and nobody has measured this machine, so confirm
or reset all three before a verdict turns on one — and that the unit's time
daemon is configured to slew rather than step while a session runs. A backwards step of
`CLOCK_REALTIME` is loss of the driver's time base: every timer that can
de-torque the machine is a difference against the clock that just moved, so the
driver answers a step by latching torque off, and nothing clears that latch but
the control process arming again. The session ends at the minimum risk
condition, by design, with the head wherever it was. That is the correct
response and a wasted run, so it is worth the check beforehand.

## The recorded traces

No command in this tree writes a trace: `--trace` belonged to the motion
commands, which are gone. The four recordings already taken are checked in
under `crates/reachy-motion/fixtures/traces` and are what that crate's bounds
are sized against; `//crates/reachy-motion:replay_test` replays them against
the figures the cog path actually runs on. The directory's `README.md` says
what each one records.

The format, which whatever records the next one has to fit: one CSV row per
control period, every joint's measured angle against the goal it was being held
to, which is the move's velocity profile at the rate it was sampled. A released
joint's goal cells are blank from the period after it went out of service and
its measured cells keep recording — that is the diagnostic of record for a
degraded run.

Two cautions that matter when reading a number off one of these files. A goal
column is what the loop *commanded*, and recordings made before the per-period
move clock carry whatever the scheduler did that night inflated into their step
sizes — no guard is ever sized against a recorded step. A measured column is the
servo's own encoder at the rate the loop read it, so velocities off it are
averages over a period.

**Keeping one.** A recording that settled a question becomes a fixture:
`crates/reachy-motion/fixtures/traces/`, a row in that directory's `README.md`
saying what it records, and a guard in `replay_test` asserting the property that
qualified it. A measurement nothing replays is folklore by the next release, and
a re-derived figure landing outside a pinned tolerance is a finding for a person
— never a tolerance widened to make the suite green. `*.csv` is gitignored
repo-wide and that directory is negated back in, so a new fixture lands with
`git add` like anything else.

The antenna phase-separation margin is the figure a recording most often argues
about. It lives in `AntennaPhaseConfig` in `reachy-motion`, is shipped by
`MotionConfig::default()`, and `ANTENNA_PHASE_SEPARATION_RAD` beside it is what
the replay suite's separation guards re-derive against: the clean pair and the
one recorded clash, either side of the bound. Proposing a wider figure means the
recording that argues for it, promoted to a fixture, and the guard moved with it
in the same commit.

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
  nothing here confirms or refutes it: no rail measurement is owed or awaited,
  so this stays a reading nobody has explained. The bit is deliberately
  excluded from the health predicate — the engage-time supply gate is what owns
  supply — so this does not stop anything today; it is an unexplained reading
  on a machine whose other readings are trusted. `reboot` is the tripwire on the
  clearing half of it: the command fails on any byte a servo still holds after
  its restart, this bit included, so a `0x01` that ever survives a reboot is a
  non-zero exit naming the servo rather than a line printed past. That is this
  observation recurring — take it to a person and do not carve the bit out.
