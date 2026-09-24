# Bench runbook — running this repo's binaries against a real unit

Imaging: brenn-pod's `docs/runbooks/reachy-end-to-end.md`. Safety:
`docs/fault-management.md`.

## What you need

- **bazel** (bazelisk) on x86_64.
- **ssh as root to the unit**, key-based.
- **`.local/reachy.conf`**, gitignored, read by every target here:

      REACHY_HOST ?= reachy00
      REACHY_SPEECH_CONFIG ?= /elsewhere/reachy-speech/speech.toml
      REACHY_HOST_PARAMS ?= /elsewhere/reachy00/host_params.textproto
      REACHY_PAYLOAD_DIR ?= user@server:/srv/reachy
      REACHY_PAYLOAD_URL ?= https://server.example/reachy

  Only make reads it; the `ssh` lines below need `export REACHY_HOST=reachy00`.
- **`.local/reachy-bench.toml`** (`BENCH_CONFIG=` overrides): copy
  `crates/reachy-bench/reachy-bench.example.toml`, fill in `[bus]`'s serial
  node.
- **A sibling brenn-pod checkout** (`BRENN_POD_DIR=<path>` otherwise) at
  `BRENN_POD_REV`, plus **podman with qemu-aarch64 binfmt**, for
  `make motion-build`'s audio binary and `make speech-run`'s provisioning;
  `REACHY_POD_BINARY=<file>` skips both.
- For a speech run, the mic array.

## Where things live

| | |
|---|---|
| `target/motion-arm64/release/` | the staged payload |
| `.local/motion-logs/`, `.local/speech-logs/`, `.local/pose-sessions/` | fetched runs, timestamped, with `.console` and `provenance.txt` |
| `/run/brenn-app/releases/motion/` | a root run's payload; a resync or boot fetch installs `releases/fetch-<stamp>`; `/run/brenn-app/current` names the active one |
| `/run/brenn-app/scratch/logs/motion/`, `scratch/logs/launch/` | `.olog` directories; consoles |
| `/run/brenn-app/conf/audio.conf` | pod link credentials |
| `/var/lib/brenn-app/` | bench configuration and self-test record |

All RAM: nothing touches the eMMC; a reboot clears it.

## Clearing the bus

`brenn-app.service` and `reachy-motiond.service` each open the servo port; the
scripts refuse rather than stop either — what runs on a device is the
operator's. A fetched unit runs `brenn-app.service` from boot; stop it before
a bench night.

    ssh root@"$REACHY_HOST" systemctl stop reachy-motiond.service
    ssh root@"$REACHY_HOST" systemctl start reachy-motiond.service   # after

**`make reachy-up` in brenn-pod is not a bench command**: it takes the bus.

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

Watch the machine; `first_motion_report` judges. Tail
`/run/brenn-app/scratch/logs/launch`: `motord_0.log`, `proc_0.log`,
`logger_proc_0.log`, plus `voice_host_0.log` and `pod_0.log` under the
production config. Ctrl-C or `curl -X POST 127.0.0.1:8080/quit` stops it; the
driver de-torques.

## The library tour

    make library-run   # build, push, play every motion, fetch, judge

Every motion in `cogs/library.names.json` but the `probe/` instruments
(`make motion-probe MOTION=<name>`), in order, at recorded pace: minutes of
unattended motion.
**Keep the space around the machine clear until it returns.** Reaching the
`timeout` fails the run; `library_tour_report` judges.

## The fetched payload

    make motion-release   # pack, publish, install now

`motion-pack` packs the operator's build, speech configuration and all: the
server admits only a unit presenting its provisioned certificate.
`motion-resync` makes the unit fetch the archive and restart
`brenn-app.service`, which tours as `app`. Every boot fetches the same URL:
**publish only a build you would let a power cycle start.** `make motion-fetch`
brings the records home. Reboot between a root run and a fetched run: they
cannot share log roots.

## The hold test, and tuning

`make motion-run`, then read `stillness`: antennas judged, head printed; never
widen the bound. Gains, profiles, `hold-probe`, `REACHY_EXPERIMENT_DIR` and
every run read so far: `docs/servo-tuning.md`.

## The speech run

    make speech-run     # provision, build, push, preflight, run, fetch, judge
    make speech-fetch   # recover a run whose terminal died

Provisioning is brenn-pod's `reachy-provision`, every time (`audio.conf` is
tmpfs; `make speech-provision` runs it alone). Ctrl-C ends it.

The **assembly directory** (`REACHY_SPEECH_CONFIG`) is `speech.toml` plus the
files it names, outside this tree, by the **payload-relative paths they will
occupy**: `pod_psk_file = "secrets/pod-psk.toml"` is
`<assembly>/secrets/pod-psk.toml`. Site values: loopback `listen_addr`;
`[stt]`/`[tts]` URLs reachable *from the robot*, never `localhost`;
`[brenn.bridge]`'s `wss://` URL and `token_file`, absent for a bus-less
pipeline; the build-staged `models/...` paths; `[wake] model`, your own head;
`[wake] phrase` — a swap is both keys in both configurations plus the file;
`[jsonl] sink = "stdout"`.

However it ends, the run is fetched; `speech_run_report` judges.

## Recording poses

    make pose-record    # provision, build, push, preflight, record, fetch
    make pose-fetch     # recover a session whose terminal died

Stop `reachy-motiond`; if the servos may hold torque, take the head's weight
and `make bench-run ARGS="off"`. Hands on the head, say `"<phrase>, <label>"`;
the robot reads each back. **Let the read-back finish before speaking again.**
Tail `recorder_0.log` and `voice_host_0.log`. Ctrl-C ends it.

`speech-record.toml` (`REACHY_RECORD_SPEECH_CONFIG`) is `speech.toml` without
`[brenn]`, with `[wake] policy = "gated"`, `[brain] mode = "echo"`, `[record]
enabled = true`; shared keys must match.

The fetch prints the `pose_session_report` command; `--extract <segment>`
drafts a clip, `--as-pose` a pose: `docs/pose-authoring.md`.

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
- **16** — `speech-record.toml` and `speech.toml` disagree on a shared key.
- **17** — the boot fetch is still retrying and installs the payload itself within 300 s.
- **18** — `brenn-app-resync` failed; see its message.

5–8, 12, 13, 17 and 18 are the remote chain's: a message and exit 1; past the
sentinel line, the launcher's.

## Open observations

- **A 545° antenna reading after a hard power cycle.** Seen once, unexplained.
  Tripwire: the self-test's `antenna-fold` case; never widen it.
- **A log recorded before a schema append cannot be read by a later build.**
  Analyze a run with the build that recorded it; `provenance.txt` names both
  sides.
