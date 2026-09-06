# Bench runbook — running this repo's binaries against a real unit

Three runs: `reachy-bench`, moving nothing; the motion test, moving the
machine; the speech run, the voice pipeline with somebody talking.
Imaging: brenn-pod's `docs/runbooks/reachy-end-to-end.md`. Safety rules:
`docs/fault-management.md`.

## What you need

- **bazel** (bazelisk; `.bazelversion` pins it) on x86_64: the device build
  cross-compiles with the pinned drop's toolchains.
- **ssh as root to the unit**, key-based; every script runs `BatchMode=yes`.
- **`.local/reachy.conf`**, gitignored, read by every target here:

      REACHY_HOST ?= reachy00
      REACHY_SPEECH_CONFIG ?= /elsewhere/reachy-speech/speech.toml
      REACHY_HOST_PARAMS ?= /elsewhere/reachy00/host_params.textproto

  Only make reads it, and it is not shell syntax: for the `ssh` lines below,
  `export REACHY_HOST=reachy00` too.
- **`.local/reachy-bench.toml`** (`BENCH_CONFIG=` overrides): start from
  `crates/reachy-bench/reachy-bench.example.toml`, fill in `[bus]`'s serial
  node.
- **A sibling brenn-pod checkout** (`BRENN_POD_DIR=<path>` otherwise):
  `make motion-build` stages its prebuilt audio binary — built once by
  `make -C ../brenn-pod/firmware reachy-pod`, or named by
  `REACHY_POD_BINARY=<path>` — and `make speech-run` invokes its provisioning
  for the pod's half of the link.
- For a speech run, the mic array plugged in.

## Where things live

| | |
|---|---|
| `target/motion-arm64/release/` | the staged payload |
| `.local/motion-logs/`, `.local/speech-logs/` | fetched runs: a timestamped directory each, with a `.console` and a `provenance.txt` naming this tree's commit and the brenn-pod revision (or `overlay:` tree) |
| `/run/brenn-app/releases/motion/` | the payload, and every process's working directory |
| `/run/brenn-app/logs/motion/`, `logs/launch/` | `.olog` directories; consoles |
| `/run/brenn-app/conf/audio.conf` | the pod's link credentials, brenn-pod's to write |
| `/var/lib/brenn-app/` | the bench's configuration and self-test record |

All RAM: no dev cycle touches the eMMC, a reboot clears it.

## Clearing the bus

`brenn-app.service` and `reachy-motiond.service` each open the servo port. The
scripts refuse (codes 3 and 4) rather than stop either: what runs on a device
is the operator's.

    ssh root@"$REACHY_HOST" systemctl stop reachy-motiond.service
    # ... the session ...
    ssh root@"$REACHY_HOST" systemctl start reachy-motiond.service

**`make reachy-up` in brenn-pod is not a bench command**: it restarts the
motion daemon and takes the bus back.

## The bench loop

    make bench-build
    make bench-config
    make bench-selftest                  # read-only: no torque, no motion
    make bench-run ARGS="off"            # the rest; ARGS="" lists them
    make bench-fetch

`bench-run` builds first, never a binary older than your tree; `--stale-ok`
right after `--run` runs an old one deliberately. An unexpected reading goes to
a person before anything is made green.

## The motion test

    make motion-run    # build, push, run on a 36 s budget, fetch, judge

**Run `make bench-run ARGS="watchdog"` and power-cycle before a unit's first
motion run.** It fails on this hardware; that failure is the record of a
watchdog trip: the servos stop still holding torque.

Watch the machine; the verdict is `first_motion_report`'s over the fetched
records. Tail from a second shell, under `/run/brenn-app/logs/launch`:
`motord_0.log`, `proc_0.log`, `logger_proc_0.log`, plus `voice_host_0.log` and
`pod_0.log` under the production config. Ctrl-C, or
`curl -X POST 127.0.0.1:8080/quit` on the unit, stops it; the driver de-torques
on the way out.

## The hold test

`make motion-run`, then read `stillness`: antennas judged, head printed.
T0: a kept, committed branch off the fix, `NEUTRAL_ANTENNAS` `[0.0, 0.0]`, sign
test flipped. Expect failure; record figures and directory under **Open
observations**. T1 is `main`; never widen the bound. A pre-raise hold is
arm-time stow, not rest. Trials, fixture: `TODO.md`'s `antenna-hold-gains`,
`antenna-hold-fixture`.

## The speech run

    make speech-run     # provision, build, push, preflight, run, fetch, judge
    make speech-fetch   # recover a run whose terminal died

The build and push are a motion run's. Provisioning is brenn-pod's
`reachy-provision`, run every time because `audio.conf` is tmpfs and a rebooted
unit needs it again; `make speech-provision` runs it alone. The far end is the
production launcher config, which adds the voice host and the audio device. No
budget: you end the run with Ctrl-C, so a non-terminal stdin is refused before
anything is provisioned or built.

The **assembly directory** is `speech.toml` plus the credentials it names, kept
outside this tree and named by `REACHY_SPEECH_CONFIG`, which names them by the
**payload-relative paths they will occupy** —
`pod_psk_file = "secrets/pod-psk.toml"` is `<assembly>/secrets/pod-psk.toml` —
because the host resolves from the payload root; an absolute path is a refused
build. The build stages each at 0600; the push refuses one rotated since. A
site's own values: loopback `listen_addr`; `[stt]`/`[tts]` URLs
reachable *from the robot*, never `localhost`; `[brenn.bridge]`'s `wss://` URL
and `token_file`, absent for a valid bus-less pipeline; four model paths
spelling the staged `models/...` names; `[jsonl] sink = "stdout"`, so events
ride `voice_host_0.log` home.

Talk to it, then Ctrl-C. However the launcher ends, the run is fetched and
`speech_run_report` judges it; one that recorded no channels is fetched too,
its console the evidence.

## Exit codes

- **3** — `brenn-app.service` holds the bus.
- **4** — `reachy-motiond.service` holds the bus.
- **5** — the payload carries no `provenance.txt`; push again.
- **6** — the stamp could not be staged: a full or read-only payload store.
- **7** — a step after the wipe failed; unfetched records are gone.
- **8** — the payload carries no launcher config for this run; push again.
- **9** — the staged payload carries no `host/speech.toml`.
- **10** — stdin is not a terminal.
- **11** — `reachy_host --check` refused the staged configuration.
- **12** — `audio.conf` is absent or empty; provisioning never reached the unit.
- **13** — the unit could not reach a speech service the config names.

5 to 8, 12 and 13 are the remote chain's and reach you as a message and exit 1.
A launcher exiting 5 itself is not the chain refusing: the chain prints a
sentinel once past its last refusal.

## Open observations

- **A 545° antenna reading after a hard power cycle.** Seen once, unexplained.
  Tripwire: the self-test's `antenna-fold` case, failing by name outside the
  turn a fold leaves. Never widen it.
- **A log recorded before a schema append cannot be read by a later build.**
  This reader binds schemas by byte equality and declares no evolution history.
  Analyze a run with the build that recorded it; `provenance.txt` names both
  sides.
