# Bench runbook — running this repo's binaries against a real unit

One run moves nothing, two move the machine, one adds the voice pipeline,
one records your hands on a de-torqued head.
Imaging: brenn-pod's `docs/runbooks/reachy-end-to-end.md`. Safety:
`docs/fault-management.md`.

## What you need

- **bazel** (bazelisk; `.bazelversion` pins it) on x86_64.
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
- **A sibling brenn-pod checkout** (`BRENN_POD_DIR=<path>` otherwise) at this
  tree's `BRENN_POD_REV`, clean or overlaid, and **podman with qemu-aarch64
  binfmt**: `make motion-build` compiles its audio binary there, refusing
  others; `make speech-run` invokes its provisioning. `REACHY_POD_BINARY=<file>`
  skips both.
- For a speech run, the mic array.

## Where things live

| | |
|---|---|
| `target/motion-arm64/release/` | the staged payload |
| `.local/motion-logs/`, `.local/speech-logs/`, `.local/pose-sessions/` | fetched runs, one timestamped directory each: a `.console` beside it, a `provenance.txt` naming this tree's commit and both brenn-pod revisions |
| `/run/brenn-app/releases/motion/` | the payload, and every process's working directory |
| `/run/brenn-app/logs/motion/`, `logs/launch/` | `.olog` directories; consoles |
| `/run/brenn-app/conf/audio.conf` | the pod's link credentials |
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

An unexpected reading goes to a person before anything is made green.

## The motion test

    make motion-run    # build, push, run on a 36 s budget, fetch, judge

**Run `make bench-run ARGS="watchdog"` and power-cycle before a unit's first
motion run.** It fails here: a watchdog trip stops the servos with torque held.

Watch the machine; the verdict is `first_motion_report`'s over the records. Tail from a second shell, under `/run/brenn-app/logs/launch`:
`motord_0.log`, `proc_0.log`, `logger_proc_0.log`, plus `voice_host_0.log` and
`pod_0.log` under the production config. Ctrl-C, or
`curl -X POST 127.0.0.1:8080/quit`, stops it; the driver de-torques.

## The library tour

    make library-run   # build, push, play every motion, fetch, judge

Every motion in `cogs/clip_library.names.json` but the `probe/` instruments,
which `make motion-probe MOTION=<name>` plays one at a time, in order, at
recorded pace: minutes of unattended motion.
**Keep the space around the machine clear until it returns.** Reaching the `timeout` fails the run;
`library_tour_report` is the verdict.

## The hold test, and tuning

`make motion-run`, then read `stillness`: antennas judged, head printed; never
widen the bound. Gains, profiles, `hold-probe`, the
`REACHY_EXPERIMENT_DIR` overlay and every run read so far:
`docs/servo-tuning.md`.

## The speech run

    make speech-run     # provision, build, push, preflight, run, fetch, judge
    make speech-fetch   # recover a run whose terminal died

Provisioning is brenn-pod's `reachy-provision`, every time (`audio.conf` is
tmpfs); `make speech-provision` runs it alone. The far end is the production
launcher config: voice host and audio device beside the motion stack. No
budget — Ctrl-C ends it.

The **assembly directory** is `speech.toml` plus the credentials it names,
outside this tree, named by `REACHY_SPEECH_CONFIG`. It names them by the
**payload-relative paths they will occupy** —
`pod_psk_file = "secrets/pod-psk.toml"` is `<assembly>/secrets/pod-psk.toml`.
Site values: loopback `listen_addr`; `[stt]`/`[tts]` URLs reachable *from the
robot*, never `localhost`; `[brenn.bridge]`'s `wss://` URL and `token_file`,
absent for a bus-less pipeline; four model paths spelling the staged
`models/...` names; `[jsonl] sink = "stdout"`.

Talk to it, then Ctrl-C. However it ends, the run is fetched and
`speech_run_report` judges it.

## Recording poses

    make pose-record    # provision, build, push, preflight, record, fetch
    make pose-fetch     # recover a session whose terminal died

Stop `reachy-motiond`; if the servos may hold torque, take the head's weight
and `make bench-run ARGS="off"`. Hands on the head, hold or move it, say what
it is; the robot reads each transcript back. **Let the read-back finish
before speaking again**: a shorter pause merges two utterances. The tail is
`recorder_0.log` and `voice_host_0.log`. Ctrl-C ends it.

`speech-record.toml` (`REACHY_RECORD_SPEECH_CONFIG`) is `speech.toml` without
`[brenn]`, with `[wake] policy = "bypass"`, `[brain] mode = "echo"`, `[record]
enabled = true`; `listen_addr`, `pod_psk_file` and `[pods]` must match.

The fetch prints the `pose_session_report` command; it writes `session.json`
and `timeline.txt` (a line per hold, move, utterance). Other segmenter flags
tune it; `--extract <segment>` drafts a clip.

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
- **14** — no staged `host/speech-record.toml`.
- **15** — no staged `bench/reachy-bench.toml`.
- **16** — `speech-record.toml` and `speech.toml` disagree on `listen_addr` or `pod_psk_file`.

5 to 8, 12 and 13 are the remote chain's: a message and exit 1; past its
sentinel line the exit is the launcher's.

## Open observations

- **A 545° antenna reading after a hard power cycle.** Seen once, unexplained.
  Tripwire: the self-test's `antenna-fold` case, failing by name outside the
  turn a fold leaves. Never widen it.
- **A log recorded before a schema append cannot be read by a later build.**
  Schemas bind by byte equality: analyze a run with
  the build that recorded it. `provenance.txt` names both sides.
