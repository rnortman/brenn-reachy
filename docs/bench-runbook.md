# Bench runbook — running this repo's binaries against a real unit

Three runs: `reachy-bench`, which moves nothing; the motion test, which moves
the machine; the speech run, the voice pipeline with a person talking to it.
Imaging the device is brenn-pod's `docs/runbooks/reachy-end-to-end.md`. Safety
rules: `docs/fault-management.md`.

## What you need

- **bazel** (bazelisk; `.bazelversion` pins it) on x86_64: the device build
  cross-compiles against the pinned drop's toolchains.
- **ssh as root to the unit**, key-based; every script runs `BatchMode=yes`.
- **`.local/reachy.conf`**, gitignored, read by every target here:

      REACHY_HOST ?= reachy00
      REACHY_SPEECH_CONFIG ?= /elsewhere/reachy-speech/speech.toml
      REACHY_HOST_PARAMS ?= /elsewhere/reachy00/host_params.textproto

  Only make reads the file — it is not shell syntax, so for the `ssh` lines
  below `export REACHY_HOST=reachy00` in your shell as well.
- **`.local/reachy-bench.toml`** (`BENCH_CONFIG=` overrides): start from
  `crates/reachy-bench/reachy-bench.example.toml`, fill in `[bus]`'s serial
  node.
- **A sibling brenn-pod checkout** (`BRENN_POD_DIR=<path>` otherwise), doing
  double duty: `make motion-build` stages its prebuilt audio binary — build it
  once with `make -C ../brenn-pod/firmware reachy-pod`, or name the artifact
  with `REACHY_POD_BINARY=<path>` — and `make speech-run` invokes its
  provisioning for the pod's half of the voice link.
- For a speech run, the mic array plugged in.

## Where things live

| | |
|---|---|
| `target/motion-arm64/release/` | the staged payload |
| `.local/motion-logs/`, `.local/speech-logs/` | fetched runs, one timestamped directory each, a `.console` beside it, `provenance.txt` naming the build |
| `/run/brenn-app/releases/motion/` | the payload, and every process's working directory |
| `/run/brenn-app/logs/motion/`, `logs/launch/` | `.olog` directories; per-process consoles |
| `/run/brenn-app/conf/audio.conf` | the pod's link credentials, brenn-pod's to write |
| `/var/lib/brenn-app/` | the bench's configuration and self-test record |

The unit's side is all RAM: no dev cycle touches the eMMC, a reboot clears
it.

## Clearing the bus

`brenn-app.service` and `reachy-motiond.service` each open the servo port. The
scripts refuse (codes 3 and 4) rather than stopping either: what runs on a
device is the operator's.

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

`bench-run` builds first, so it cannot run a binary older than your tree;
`--stale-ok` first after `--run` runs an old one deliberately. An unexpected
reading goes to a person before anything is made green.

## The motion test

    make motion-run    # build, push, run on a 30 s budget, fetch, judge

**Run `make bench-run ARGS="watchdog"` and power-cycle before a unit's first
motion run.** It fails on this hardware, and that failure is the record of a
watchdog trip: the servos stop still holding torque.

Watch the machine; the target's verdict is `first_motion_report`'s over the
fetched records. Tail from a second shell, under
`/run/brenn-app/logs/launch`: `motord_0.log`, `proc_0.log`,
`logger_proc_0.log`, plus `voice_host_0.log` and `pod_0.log` under the
production config. Ctrl-C, or `curl -X POST 127.0.0.1:8080/quit` on the unit,
stops the run; the driver de-torques on the way out.

## The speech run

    make speech-run     # provision, build, push, preflight, run, fetch, judge
    make speech-fetch   # recover a run whose terminal died

The build and push are a motion run's. The provision step is brenn-pod's
`reachy-provision`, run for you every time: `audio.conf` is tmpfs, so a
rebooted unit needs it again and no command of yours says so;
`make speech-provision` runs that step alone. The far end is the production
launcher config, which adds the voice host and the audio device, and there is
no budget: you end the run with Ctrl-C, so a non-terminal stdin is refused
before anything is provisioned or built.

The **assembly directory** is `speech.toml` plus the credentials it names, kept
outside this tree and named by `REACHY_SPEECH_CONFIG`. It names them by the
**payload-relative paths they will occupy** —
`pod_psk_file = "secrets/pod-psk.toml"`, the file at
`<assembly>/secrets/pod-psk.toml` — because the host resolves from the payload
root; an absolute path is a refused build. The build stages each at 0600; the
push refuses one rotated since. Shape only, the values being a site's own:
loopback `listen_addr`; `[stt]`/`[tts]` URLs reachable *from the robot*, never
`localhost`; `[brenn.bridge]`'s `wss://` URL and `token_file`, absent for a
valid bus-less pipeline; four model paths spelling the staged `models/...`
names; `[jsonl] sink = "stdout"`, so the pipeline's events ride
`voice_host_0.log` home.

Talk to it, then Ctrl-C. However the launcher ends, the run is fetched and
`speech_run_report` judges it — why it ended is the report's question. A run
that recorded no channels is fetched too: its console is the evidence.

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
- **12** — `audio.conf` is absent or empty; provisioning did not reach the unit.
- **13** — the unit could not reach a speech service the config names.

5 to 8, 12 and 13 are the remote chain's and reach you as a message and exit 1.
A launcher that itself exits 5 is not the chain refusing: the chain prints a
sentinel once past its last refusal.

## Open observations

- **A 545° antenna reading after a hard power cycle.** Seen once, unexplained.
  Tripwire: the self-test's `antenna-fold` case, failing by name outside the
  turn a fold leaves. Never widen that bound.
- **A log recorded before a schema append cannot be read by a later build.**
  This reader binds schemas by byte equality and declares no evolution history.
  Analyze a run with the build that recorded it; `provenance.txt` names it.
