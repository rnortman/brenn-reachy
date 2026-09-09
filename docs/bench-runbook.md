# Bench runbook — running this repo's binaries against a real unit

One run moves nothing, two move the machine, one adds the voice pipeline.
Imaging: brenn-pod's `docs/runbooks/reachy-end-to-end.md`. Safety:
`docs/fault-management.md`.

## What you need

- **bazel** (bazelisk; `.bazelversion` pins it) on x86_64: the device build
  cross-compiles with its toolchains.
- **ssh as root to the unit**, key-based; every script runs `BatchMode=yes`.
- **`.local/reachy.conf`**, gitignored, read by every target here:

      REACHY_HOST ?= reachy00
      REACHY_SPEECH_CONFIG ?= /elsewhere/reachy-speech/speech.toml
      REACHY_HOST_PARAMS ?= /elsewhere/reachy00/host_params.textproto

  Only make reads it, and not as shell syntax; for the `ssh` lines below,
  `export REACHY_HOST=reachy00` too.
- **`.local/reachy-bench.toml`** (`BENCH_CONFIG=` overrides): copy
  `crates/reachy-bench/reachy-bench.example.toml`, fill in `[bus]`'s serial
  node.
- **A sibling brenn-pod checkout** (`BRENN_POD_DIR=<path>` otherwise):
  `make motion-build` stages its prebuilt audio binary — built once by
  `make -C ../brenn-pod/firmware reachy-pod`, or `REACHY_POD_BINARY=<path>` —
  and `make speech-run` invokes its provisioning for the pod's half.
- For a speech run, the mic array.

## Where things live

| | |
|---|---|
| `target/motion-arm64/release/` | the staged payload |
| `.local/motion-logs/`, `.local/speech-logs/` | fetched runs, one timestamped directory each: a `.console` beside it, a `provenance.txt` naming this tree's commit and the brenn-pod revision (or `overlay:` tree) |
| `/run/brenn-app/releases/motion/` | the payload, and every process's working directory |
| `/run/brenn-app/logs/motion/`, `logs/launch/` | `.olog` directories; consoles |
| `/run/brenn-app/conf/audio.conf` | the pod's link credentials, brenn-pod's to write |
| `/var/lib/brenn-app/` | the bench's configuration and self-test record |

All RAM: no dev cycle touches the eMMC, a reboot clears it.

## Clearing the bus

`brenn-app.service` and `reachy-motiond.service` each open the servo port. The
scripts refuse rather than stop either: what runs on a device is the
operator's.

    ssh root@"$REACHY_HOST" systemctl stop reachy-motiond.service
    ssh root@"$REACHY_HOST" systemctl start reachy-motiond.service   # after

**`make reachy-up` in brenn-pod is not a bench command**: it restarts the
motion daemon and takes the bus.

## The bench loop

    make bench-build
    make bench-config
    make bench-selftest                  # read-only: no torque, no motion
    make bench-run ARGS="off"            # the rest; ARGS="" lists them
    make bench-fetch

`bench-run` builds first, never a binary older than your tree; `--stale-ok`
after `--run` runs an old one deliberately. An unexpected reading goes to
a person before anything is made green.

## The motion test

    make motion-run    # build, push, run on a 36 s budget, fetch, judge

**Run `make bench-run ARGS="watchdog"` and power-cycle before a unit's first
motion run.** It fails here; that failure is the record of a watchdog trip:
servos stop still holding torque.

Watch the machine; the verdict is `first_motion_report`'s over the records. Tail from a second shell, under `/run/brenn-app/logs/launch`:
`motord_0.log`, `proc_0.log`, `logger_proc_0.log`, plus `voice_host_0.log` and
`pod_0.log` under the production config. Ctrl-C, or
`curl -X POST 127.0.0.1:8080/quit`, stops it; the driver de-torques.

## The library tour

    make library-run   # build, push, play every motion, fetch, judge

Every motion in `cogs/clip_library.names.json` but the `probe/` instruments,
which `make motion-probe MOTION=<name>` plays one at a time under
`probe-log-<stamp>`, in order, at recorded pace: minutes of unattended motion.
**Keep the space around the machine clear until it returns.** The tour quits the launcher itself; the
`timeout` is a backstop the sender sizes, and reaching it fails the run.
`library_tour_report` judges the records: every motion asked once in order, no
fault, every window moving the machine, no gap in the samples. It prints
residual, lag, peak step and antenna separation for **Open observations**.

## The hold test, and tuning

`make motion-run`, then read `stillness`: antennas judged, head printed; never
widen the bound. The gains and profile ladders, the `hold-probe` command, the
`REACHY_EXPERIMENT_DIR` overlay and the record of every run read so far are
`docs/servo-tuning.md`.

## The speech run

    make speech-run     # provision, build, push, preflight, run, fetch, judge
    make speech-fetch   # recover a run whose terminal died

Build and push are a motion run's. Provisioning is brenn-pod's
`reachy-provision`, every time, because `audio.conf` is tmpfs;
`make speech-provision` runs it alone. The far end is the production launcher
config: voice host and audio device beside the motion stack. No budget — Ctrl-C
ends it, so a non-terminal stdin is refused first.

The **assembly directory** is `speech.toml` plus the credentials it names,
outside this tree, named by `REACHY_SPEECH_CONFIG`. It names them by the
**payload-relative paths they will occupy** —
`pod_psk_file = "secrets/pod-psk.toml"` is `<assembly>/secrets/pod-psk.toml`,
since the host resolves from the payload root; an absolute path is a refused
build. The build stages each at 0600; the push refuses one rotated since. Site
values: loopback `listen_addr`; `[stt]`/`[tts]` URLs reachable *from the
robot*, never `localhost`; `[brenn.bridge]`'s `wss://` URL and `token_file`,
absent for a bus-less pipeline; four model paths spelling the staged
`models/...` names; `[jsonl] sink = "stdout"`, so events ride
`voice_host_0.log` home.

Talk to it, then Ctrl-C. However it ends, the run is fetched and
`speech_run_report` judges it; one that recorded no channels too, its console
the evidence.

## Exit codes

- **3**, **4** — `brenn-app.service`, `reachy-motiond.service` holds the bus.
- **5** — no `provenance.txt` in the payload; push again.
- **6** — the stamp could not be staged: a full or read-only store.
- **7** — a step after the wipe failed; unfetched records are gone.
- **8** — no launcher config for this run; push again.
- **9** — no staged `host/speech.toml`.
- **10** — stdin is not a terminal.
- **11** — `reachy_host --check` refused the staged configuration.
- **12** — `audio.conf` absent or empty; provisioning never landed.
- **13** — a speech service the config names is unreachable from the unit.

5 to 8, 12 and 13 are the remote chain's: a message and exit 1. A launcher
exiting 5 is not the chain refusing — the chain prints a sentinel past its last
refusal.

## Open observations

- **A 545° antenna reading after a hard power cycle.** Seen once, unexplained.
  Tripwire: the self-test's `antenna-fold` case, failing by name outside the
  turn a fold leaves. Never widen it.
- **A log recorded before a schema append cannot be read by a later build.**
  Schemas bind by byte equality, with no evolution history: analyze a run with
  the build that recorded it. `provenance.txt` names both sides.
