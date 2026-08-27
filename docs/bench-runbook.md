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
  shipped default with the reasoning beside it, and fill in the one thing an
  example cannot know: the serial node under `[bus]`. Nothing in that file
  configures motion — the bench moves nothing.

  A copy predating the `[datum]` table's removal fails parse naming `datum`.
  Delete the table; nothing reads it. The datum lives in the self-test record,
  written by the `datum` case from what the servos' homing offset registers
  actually held.

## Where things live

| | |
|---|---|
| `.local/reachy.conf` | the unit's hostname. Gitignored |
| `.local/reachy-bench.toml` | the unit's bench configuration. Gitignored: it holds this machine's serial node |
| `.local/records/` | fetched self-test records, and wherever you park retrieved traces |
| `target/bench-arm64/release/reachy-bench` | the device binary `bench-build` produces |
| `target/motion-arm64/release/` | the motion payload `motion-build` stages: both binaries and every configuration file the three processes read, laid out the way their relative paths expect |
| `.local/motion-logs/` | fetched run logs, one directory per fetch, each holding the writer's run directory and a `provenance.txt` naming the build that recorded it, and `<fetch>.console` beside each one: the launcher's per-process console files, and for a run also its own console stream and the unit's clock discipline read either side of it |
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

`deploy-motion.sh --run` appends three codes of its own to that remote chain,
and they are part of the same contract. Two of them fire before anything is
emptied: **5** — the payload on the unit carries no `provenance.txt`, so the
run's records could not name their build, and the fix is another `--push`; **6**
— the stamp is there but copying it to its staging path failed, which is a full
or read-only payload store. On both, nothing was started and nothing was
emptied, so the previous run's records are still on the unit and still
fetchable. **7** — one of the steps that run after the wipe began failed: the log
root's own `mkdir`, the stamp's move into it, the launcher console directory's
wipe and recreate, or the `cd` into the release. Treat a previous run's
unfetched records as gone; the launcher was not started. All three miss
`timeout`'s codes (124–127, 137) and ssh's 255; a launcher chain that exits with
exactly 5, 6 or 7 is read as the refusal instead, which fails the run either
way.

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

The servos' own Bus Watchdog is what a driver that was killed, crashed or
unplugged with the machine under torque leaves behind: the register counts bus
silence in 20 ms units, and a session arms it at 10 — 200 ms — on every servo,
every engagement. What a trip does is established by the `watchdog`
self-test below: the servo **stops, and keeps holding torque**. It is a
stop-motion net and not a de-energize one, so nothing automatic answers the
torque half of a killed driver — only the driver's own controlled wind-down
de-torques this machine. The self-test asserts a release because the fault
policy requires one, and it fails here by design; that failure is the record of
the defect and is not to be made green.

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
5. two timeouts of true silence do. The three readings of that state — the
   register, the torque, and what a goal rewrite comes back with — are all taken
   and printed before any of them is judged, so a run always tells you what the
   trip did to the torque even when something else about it was wrong. Then:
   the register reads `0xff`, the torque reading is reported and asserted
   *released*, and the goal rewrite is refused with its error field printed
   whole (`0x87` on this unit — the Access code, plus the standing
   input-voltage alert bit every servo here carries),
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

Assertion 5's torque half **fails on this unit, and that is the current known
state.** Run on 2026-08-25, servo 17: the register latched at `0xff`, the goal
rewrite was refused with `0x87`, and Torque Enable read **1** — the servo
stopped and went on holding. The vendor's manual describes exactly that, a halt
with the motion profile applied as zero rather than a release. So an armed Bus
Watchdog on this machine is a stop-motion net, not a de-torque net, and the
fault policy's killed-driver backstop premise does not hold.

The assertion stays as it is. Do not flip it to match the vendor: this is a
policy-level finding awaiting its own cycle, and a test edited to pass is that
finding retracted. Until then the thing that actually de-torques on a controlled
exit is the driver's own SIGINT/SIGTERM wind-down.

Run this **before the first motion run**, and power-cycle afterwards if a
read-only sweep comes next: this command leaves the register at 0 on the servo it
addressed, but a session leaves 10 on all nine.

## The motion test

The run that moves the machine. Not a bench command and never will be: motion
lives in the cog system, so a motion test on a unit is a run of that system.

**The motion payload has completed runs on `reachy00`, and every verdict so far
is red.** The runs are real — the launcher comes up, the driver holds its grid,
the logger writes, and the analyzer judges. Two of the four moved the machine
through the whole gesture and released it stowed with no fault firing; every red
mark in all four came from the analyzer rather than from the machine, and some
of those marks were the report reading the wrong instant. That verdict is still
a standing human-review item and not a target to make green: what follows is the
procedure and the gates for the run that moves the machine, and the bring-up
rule applies — an unexpected reading goes to a person before anything is changed
to accommodate it. What has been seen so far, and what of it has since been
settled, is under **Open observations**, "the first motion runs end red".

**Before the first motion run on a unit, run the `watchdog` self-test** (above)
and then power-cycle. Not to pass it — on this hardware it fails, and that
failure is the standing record of what the trip does: a killed or crashed driver
*does* leave the machine holding torque, because the 200 ms watchdog every
session arms stops the servos without releasing them. Run it to confirm that
transcript is still what this unit produces. The layer that de-torques is the
driver's controlled wind-down, and the launcher's own stop gesture is how you
reach it.

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

    make motion-run         # build, push, run, fetch the records, judge them

One command. It builds the aarch64 payload, pushes it into the unit's RAM, makes
the log directory, starts the run and streams its console here, stops it when the
budget is up, fetches the `.olog` directories into `.local/motion-logs/` under a
timestamped name, and runs `first_motion_report` over the run directory it
brought back. **The target's verdict is the report's.** Nothing about the run is
judged by the clock — the budget only has to be long enough for the gesture.

**Every fetched records directory says which build recorded it.** The build
records the commit it staged the payload from, inside the payload; the push turns
that into a stamp — the commit, which of the two answered (`commit_source=build`
is the payload's own record, `commit_source=push` the pushing tree's HEAD because
the payload named none), the pushing tree's HEAD as `pushed_from`, whether the
tree was dirty, and whether the age check was skipped. The stamp travels in the
payload, so it lands on the unit's tmpfs with what it describes, and the run
copies it into the log root, so it comes home at the root of the records as
`provenance.txt`. A log is readable by the build that wrote it and no other, so
reading an older run is
`git switch --detach $(sed -n 's/^commit=//p' <fetch>/provenance.txt)`. What the
other fields cost: `dirty=yes` means uncommitted edits at push time, so the
commit does not fully describe what ran; `dirty=unknown` means the repository
would not answer that question at all; `age_unchecked=yes` means `--stale-ok`
skipped the freshness refusal, so the payload may predate the commit; a
`pushed_from` different from `commit` means the tree moved between the build and
the push, and `commit` is the one that built. A push that cannot state its commit
refuses, and so does a run that finds no stamp in the payload — push again. That
refusal happens before the log root is emptied, so an unfetched previous run is
still there to fetch.

A run brings back more than the records. The launcher's whole console directory
lands in `<fetch>.console` beside them, the streamed console is teed into
`run-console.log` there, and the unit's time-daemon state is read into
`clock-before.txt` and `clock-after.txt` either side of the run. The driver's
console is evidence about the *log* rather than about the machine: it counts the
datagrams the driver took off its own socket, so the report is handed it with
`--console` and says so when the recorded trail is shorter than what the driver
counted. A console that does not come back costs the cross-check and never the
run.

`motion-run` chains `motion-deploy`, which chains `motion-build`, so the blessed
entry point cannot run a payload older than your tree. `make motion-deploy` and
`make motion-fetch` are still there for the manual path — a run you start by hand
from the appendix at the end of this section.

Both refuse while `brenn-app.service` or `reachy-motiond.service` is running —
either of them holds the servo bus — and the refusal names the service and the
command, exactly as the bench's does. For a push that question is advisory: a
push starts nothing, and what keeps two claimants off the bus is the driver's
exclusive open of the port. For a run it is binding, because the question and the
launcher go over one ssh invocation, so nothing can take the bus between the
answer and the start.

The payload lands at `/run/brenn-app/releases/motion` — tmpfs, cleared by a
reboot, and the working directory the launcher is started from, because every
path in its config, and every configuration file each process reads, is named
relative to it. The records land under `/run/brenn-app/logs/motion` and the
processes' console output under `/run/brenn-app/logs/launch`, also RAM: pull
what you want off with the run.

`make motion-run` empties `/run/brenn-app/logs/motion` before it starts, so what
it fetches and judges is that run's records and nothing else. The logger writes a
directory per run under that root, and the report reads the newest one: left to
accumulate, a run that wrote nothing would be judged — and passed — on an earlier
run's log. It empties `/run/brenn-app/logs/launch` for the same reason: the
launcher numbers its files per run and never overwrites, so a directory left to
accumulate holds several runs' `motord_N.log` and the console can no longer be
attributed to the run it came with. The cost is that a run refused before its
fetch leaves its records for the next run's clear; `make motion-fetch` them
first if you want them.

### Starting, watching, stopping

`make motion-run` starts it, on a thirty-second budget, and stops it by letting
the budget expire — `timeout --signal=INT`, the same signal a Ctrl-C sends. The
launcher runs on a pty, so its console streams into your terminal as it goes and
a local Ctrl-C reaches the remote process group. Started from something that is
not a terminal — a wrapper, the far end of a pipe — there is no pty, and the
script says so as it starts: the run still stops at the budget, but a Ctrl-C then
kills only the wrapper, and `ssh root@<unit> pkill -x simplelaunch` is what stops
the run itself.

**Keep your eyes on the machine while it runs.** That is what standing at the
unit is for; the verdict comes from the log afterwards.

The launcher redirects each process's console into its own file under
`/run/brenn-app/logs/launch` on the unit — `motord_0.log`, `proc_0.log`,
`logger_proc_0.log`, a second run in the same log directory writing
`motord_1.log` and so on. What streams into your terminal is the launcher's own
output; the per-process files are written on the unit and fetched with the run,
and a `tail -f` on one of them from a second shell is how you watch a process
while the machine is still.

**What the driver's console holds is counters, not narration.** It prints a
`key=value` summary every five seconds — datagrams taken off its socket, aux
offers refused, confirmation misses — and nothing about which transaction did
what. The commissioning survey's verdict is not there and never was; it is the
`commission_failed` report row that says why a session parked, and the driver's
counters are how the analyzer decides whether the recorded trail is complete.

Aborting `make motion-run` with Ctrl-C leaves the machine safe — the driver winds
down on the signal — and fetches nothing. Re-run it.

**There is no start order to get right, and none to get wrong.** The launcher
spawns all three at once, in whatever order it happens to walk them, and each is
safe alone: a driver with nobody talking to it reaches the minimum risk
condition inside its startup window and says so; a control loop whose driver is
not up yet finds the servos absent — its first presence ping has an 800 ms
delivery budget against a port bound milliseconds after the fork — and parks the
session without ever writing torque, which is a dead run and never a hazard; a
logger that starts late costs early records and nothing else.

**Stopping is one gesture, and any way of stopping is safe.** The budget's
SIGINT, a Ctrl-C in the launcher's terminal, or
`curl -X POST 127.0.0.1:8080/quit` from anywhere on the unit — the same signal
by three routes. The launcher SIGINTs all three children: the cog processes stop
cleanly, and the driver de-torques the machine, confirms it, prints what it did,
and exits — well inside the 3 s before the launcher escalates to SIGTERM. **That
wind-down is what de-torques this machine**, and it is why every controlled exit
is a safe one.

What a signal never reaches — SIGKILL, a crash, a yanked cable, a wedged host —
is covered by the servos' own Bus Watchdog, armed by the session at 200 ms. On
this unit a trip **stops** the servos without releasing them: the head holds its
pose rather than going limp. The **`watchdog` self-test** section above has the
reading and what it means; run that self-test plus a power cycle before the first motion
run.

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
that window is the normal shape of a good run, not a wedged one. Nothing narrates
the survey while it runs — a `tail` on the driver's console shows its five-second
counters and no more — and a survey that fails ends the session `PARKED` with a
`commission_failed` row in the log saying which kind, which servo, and the one
number that decided it.

Watch for it with your eyes on the machine and your verdict from the log.
`first_motion_report` over the fetched records is the verdict: the phase
sequence in order, zero tick faults, no `bus_failure` and no missed cycle slot, a
release with evidence, every aux transaction back `OK`, and the measured half
printed either way — per-cycle jitter, the driver's own per-cycle work and
aux-exchange spans, read losses, tracking lags beside the replay suite's pinned
headroom, the antennas where they were at the end of the schedule, the health
rotation's readings, and a per-channel census. It prints the measurements whether
it passes or fails, because a first run that fails is exactly the run whose
numbers somebody needs. Its own test runs it over the S1 scenario's deterministic
log, so the analyzer was proved before any hardware log existed.

**A finding is one fact, and it says which one.** Missed cycle slots are one
finding for the run — total, rate, the worst report, what the cycle before it
spent, and how many of the skips followed a cycle carrying out-of-band bus work —
and a heartbeat gap a skip accounts for is not counted a second time. The latched
error byte is one finding naming the servos. A refused or mismatched transaction
is printed with its identity: correlation number, operation, servo, register, and
the value in the register's own units. **No budget is folded into any of this**:
one missed slot still makes the run red, and `TODO(cycle-skip-budget)` is where a
measured, reviewed, explained rate would go if the answer turns out to be that
one is acceptable.

The antennas are judged where the machine was let go of — the last valid pose at
or before the first verified torque-off write — and not where the script's
schedule ran out. Those are different instants: the profiled antenna motion is
still finishing when the schedule ends, and the wind-down closes it out. The
schedule-end deviation is printed as a note every run and judged never. A run
with no verified release in the log falls back to the schedule's end *and* raises
the missing release as its own finding, because a moving run with no release
evidence is the worse fact.

File the report and the log with the run record, and check that the records
directory names the commit: `cat <fetch>/provenance.txt`. That is what makes the
log readable later — a `dirty=yes`, `dirty=unknown` or `age_unchecked=yes` stamp,
or a `pushed_from` that differs from `commit`, is worth writing down beside the
run record, because the commit alone then does not say what ran.

Four things to check on the unit the first time, none of which any test here can
answer. Two of them `reachy00`
has now answered: the log directory's filesystem accepts direct I/O on that
kernel — the writer opens every file `O_DIRECT | O_DSYNC`, a refusal is a failed
open at the first file, and it has written records on this unit — and the
aarch64 binaries run at all, which executed runs settle for this unit and
`make check-device` half-answers for a build (it asserts their instruction set is
this CPU's, but executes nothing). One is live, and one is now recorded for you
to read. Live: that the arrival tolerances the report judges by are the ones you
want — `ARRIVAL_OFFSET_M`, `ARRIVAL_TURN_RAD` and `ARRIVAL_ANTENNA_RAD` in
`cogs/first_motion_report.rs` were sized from the mechanism on paper by an agent
and nobody has measured this machine. The recorded runs no longer press on
`ARRIVAL_ANTENNA_RAD` — judged at the release, both moving runs came in inside a
twentieth of it — so it is a footnote again rather than the thing a verdict turns
on, and it is confirmed or reset because somebody measured the mechanism, never
because a verdict wanted it wider. Recorded: whether the unit's time
daemon is configured to slew rather than step while a session runs. A run now
captures the daemon's state either side of itself into `clock-before.txt` and
`clock-after.txt`, so the answer arrives with the records; read it, because a
capture nobody reads settles nothing. A backwards step of
`CLOCK_REALTIME` is loss of the driver's time base: every timer that can
de-torque the machine is a difference against the clock that just moved, so the
driver answers a step by latching torque off, and nothing clears that latch but
the control process arming again. The session ends at the minimum risk
condition, by design, with the head wherever it was. That is the correct
response and a wasted run, so it is worth the check beforehand.

### Appendix: starting a run by hand

`make motion-run` automates everything below, and this is what it automates —
for a run you want to hold open longer than the budget, or to stop yourself, or
to watch from the unit. It assumes the payload has been pushed
(`make motion-deploy`) and that the shipped `log_root_dir` is in force; if the
staged `cogs/robot_logger.textproto` names another root, use that one instead of
`/run/brenn-app/logs/motion`.

On the unit, as root. The launcher resolves every path in its config against its
working directory, so it is started from the release root and nowhere else. It
runs in the foreground:

    cd /run/brenn-app/releases/motion && ./simplelaunch robotcpu.textproto \
        --logdir /run/brenn-app/logs/launch

Watch it, one per shell:

    tail -f /run/brenn-app/logs/launch/motord_0.log        # the driver
    tail -f /run/brenn-app/logs/launch/proc_0.log          # the control loop
    tail -f /run/brenn-app/logs/launch/logger_proc_0.log   # the logger

Stop it with Ctrl-C in the launcher's terminal, or from anywhere on the unit:

    curl -X POST 127.0.0.1:8080/quit

**Before you power the unit down, look at the log root while it is still up:**

    ls -l /run/brenn-app/logs/motion/*/

A run directory holding a non-zero-length `.olog` is the only evidence that the
writer got as far as records; that directory is tmpfs and a power-down takes the
answer with it. `make motion-fetch` refuses a fetch that brought no records
rather than printing a report command over nothing, but by then the machine may
be off and the run unrepeatable — so the observation belongs here, while it is
still a question you can act on. `make motion-run` has no such step: it fetches
while the unit is still up.

Then bring the records back and judge them:

    make motion-fetch
    bazel run //cogs:first_motion_report -- \
        --console .local/motion-logs/<fetch>.console \
        .local/motion-logs/<fetch>/<run>

`motion-fetch` brings the launcher's console directory back beside the records,
and `--console` is what lets the report cross-check the recorded trail against
the driver's own counters. Drop the flag if no driver console came back — the
report refuses a console path holding none, and it refuses one holding several
driver logs, which is what a launch directory left to accumulate across runs
looks like. A hand-started run has to empty `/run/brenn-app/logs/launch` itself
to avoid that; `make motion-run` does it for you. It also copies the push's
provenance stamp into the log root, which a hand-started run has to do itself if
the fetched records are to name their build:

    cp /run/brenn-app/releases/motion/provenance.txt \
        /run/brenn-app/logs/motion/provenance.txt

A driver console that came back
holding no counter summary — a run that ended before the driver's first
five-second line — is not a refusal: the report says the cross-check was not
made and judges the log anyway, because those are exactly the runs worth
reading.

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
- **The first motion runs end red.** The four `make motion-run` runs on
  `reachy00` on 2026-08-26 all produced a red report, in two shapes. The first
  shape is a session that parks at the start: `STARTING → PARKED` rather than
  `RESTING`, zero motion scripts reaching the session, one aux transaction back
  `REFUSED`, and nothing ever energised — zero samples carrying both a reading
  and a setpoint, head and antenna lag 0.0000 rad. The second shape is a session
  that commissions, wakes and moves: it reached both targets, stowed to within
  0.0261 rad on the worst of nine joints, and confirmed torque off on all nine
  rows, and was called red by the report anyway. **No fault fired in any of the
  four** — every red mark came from the analyzer, not from the machine.

  What has since been settled, and what has not:

  - *Why the parked runs parked is unrecoverable, and the refusal is not what it
    was first read as.* No verdict was recorded at the time. The refused
    transaction in each was recovered by identity: corr 0, `PING` servo 10, the
    first transaction of the commissioning survey. For *that* identity the
    driver has no pre-bus decline that can fire — servo 10 is an addressed id
    (`crates/reachy-motion/src/arm.rs`, `SERVO_IDS`), a ping reaches neither the
    value-shape nor the register refusals
    (`crates/reachy-motord/src/aux.rs`), and of the bus layer's declining arms
    only `Encode` is on the ping path, which fails for a non-unicast id or a
    too-small buffer and neither holds (`crates/dxl-proto/src/encode.rs`). So
    the lone logged `REFUSED` reads as the busy-slot answer to a delivery
    re-issue that arrived while the original still sat in the driver's slot
    (`crates/reachy-motord/src/tick.rs`) — slot pacing, the transport doing its
    job. The evidence differs per run: `144512Z`'s log records the re-issue
    byte for byte; `144804Z`'s lost the duplicate to the head truncation, and
    its corroboration is the driver's counters (`session_cmds=2, taken=2,
    aux_refused=1` — two datagrams offered, one refused, and a logged outcome
    that per the counter semantics only the busy answer produces). Those
    counters carry the shortfall caveat the next observation states.

    A delivery retry re-issues only an *unanswered* datagram, so what this
    points at is the survey's **first ping drawing no answer in time** in two of
    four runs on the same unit — an open question about the bus and servo 10,
    not a driver-side decline. What parked the session is still unrecoverable:
    each run's log is short of the driver's own datagram count at the front
    (`TODO(olog-head-capture)`), so the transaction that decided the park may be
    one the log never held. `commission_failed` now narrates the verdict, so the
    next parked run says why itself.
  - *The antennas did reach stow.* The bit-identical 0.3217 rad `AntennaRight`
    figure in both moving runs is the deterministic endpoint of the profiled
    antenna motion at the moment the script's schedule ran out, which is where
    the report used to judge it. Measured at the release instead, the two runs
    read 0.0050 / 0.0057 rad and 0.0004 / 0.0026 rad against a 0.1 rad
    tolerance. Not an anomaly; what remains open is that the antennas need
    longer than the scripted 3 s stow step, with the servo profile behind that
    still unmeasured (`TODO(session-servo-profile)`).
  - *The gap and jitter anomalies persist under motion.* 132–134 one-slot skips
    per moving run and one two-slot skip, against 59 per parked run; worst read
    jitter 8.63 ms, mean 4.771 ms, against a 20 ms grid. The suspicion is
    out-of-band bus work pushing a cycle past its slot — the parked runs' skips
    arrive on the 500 ms health-poll cadence — but nothing in those four runs
    measured a cycle, so the suspicion is unconfirmed. The driver now measures
    per-cycle work and aux-exchange spans and publishes them; a run taken with
    that in place is what sizes `TODO(cycle-skip-budget)`, and until a person
    reviews such a run, one missed slot is still red.

  The tripwire is the report itself — every run reproduces its verdict, and
  `make motion-run` exits on it. A change in the shape goes to a person, and no
  arrival tolerance is resized to make a run green.
- **The pin sweep's write read back one quantum different — settled.** Once, in
  the last moving run: corr 131, servo 15 `GOAL_POSITION`, wrote -1.0032 rad and
  read back -1.0017 rad, a difference of 0.0015 rad, which is one servo position
  count (0.088°). The mechanism is the engage sequence's pin sweep: it writes
  each joint's measured angle to the goal register **while torque is off**, and
  with torque off this platform's goal register mirrors the present position —
  so the read-back is the servo's own count at that instant, not the write. The
  sweep neither depends on the write sticking nor judges the answer, so at some
  nonzero rate the mirrored count differs from the one written and a read-back
  that nobody reads reports a mismatch.

  The earlier reading of this — "an angle whose commanded value fell between two
  counts" — was impossible: the comparison is count-exact on the same rounded
  count on both sides, so a value between counts cannot produce a difference.
  The resolution is in the transaction vocabulary rather than in the report or
  the comparison: the pin sweep now asks for the unverified write
  (`AuxOpKind::write_reg` — value on the wire, acknowledgement taken, nothing
  read back), which is the transaction a sequence that documents it will not
  judge a read-back should be issuing. The verified write's count-exact
  comparison did not move and no tolerance was introduced anywhere; the report
  still calls any `VERIFY_MISMATCH` a failure, and after this change every
  verified `GOAL_POSITION` write is gone from the command paths, so that rule
  has no by-design exception left to fire on.
- **The recorded trail is shorter than the run.** In all four runs the `.olog`
  holds fewer `SessionCmdChan` datagrams than the driver counted taking off its
  own socket — one short in each parked run, ten and four short in the moving
  runs — and each moving run also holds an outcome whose request is missing. The
  mechanism is confirmed in the pinned Clockwork drop's source, and it is the
  start of the run that is lost: the supervisor starts the logger and the payload
  processes in no particular order and waits for no readiness, the logger opens
  its subscriptions lazily on a poll after it opens the log, and on attaching to
  a channel of the kind every channel here is, the observer snaps its cursor to
  the write head — skipping whatever the ring is still holding. Enlarging the
  ring therefore recovers nothing: retention is not the limit, the attach
  position is. Tracked as `TODO(olog-head-capture)`, which is where the two
  routes out of it are written down. The report cross-checks its joined totals
  against the driver's counters and calls a shortfall a finding **against the
  log, not against the run**; that check is the tripwire, and it is why
  `--console` is worth handing it. Reading a truncated trail as a complete one is
  the failure this guards against.
- **A log recorded before a schema append cannot be read by a build after it.**
  Not an anomaly — a determination, recorded here because it costs a run's
  evidence, and the limit is this *reader's* rather than the log format's. The
  `.olog` does store each channel's full serialized schema, and the upstream C++
  and Python readers of the pinned drop decode an older recording through an
  upgrader keyed on evolution declarations in the `.clk` grammar. The Rust reader
  this repo analyses logs with binds none of that: byte-equality of the schema
  definition plus an exact-size decode are its only schema facilities, and no
  schema in this tree declares an evolution history. So an appended enum value
  fails the binding check and an appended field additionally changes the wire
  size, and no relaxation of the check could decode an old payload without a
  decode engine behind it. The four runs above predate the `reports.clk`,
  `driver/health.clk` and `motion/bus_txn.clk` appends and need a build from
  before them to read. Analyze a run's records with the build that recorded them,
  or before you append — and `provenance.txt` at the root of a fetched records
  directory is what names that build. Closing the gap instead would take an
  upstream ask plus history declarations in these schemas; nothing here forecloses
  it, and nothing here has designed it.
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
