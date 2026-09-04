# Changelog

All notable changes to brenn-reachy are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project aims to adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0/).

## [Unreleased]

Nothing has been released, and nothing here has driven a motor.

### Added

- **Repository scaffolding.** Apache-2.0 license and notice, charter, and TODO
  ledger; secret-scanning commit and push gates wired by `make setup-hooks`; CI
  that independently scans the tree and runs the same `make check` gate a
  developer runs.
- **The Cargo workspace and its five crates**, declared with their dependency
  edges and their reasons for existing: `dxl-proto` (servo wire protocol),
  `reachy-kin` (head kinematics and travel envelope), `reachy-motion`
  (trajectories, per-tick control, arm and disarm sequences), `reachy-bus` (the
  one I/O layer), and `reachy-bench` (bench binary and self-test registry). Each
  is a skeleton at this point — the module headers state the contract each will
  hold to.

- **The robot says its critical alerts out loud.** An alert the host's table
  raises Critical now carries a sentence for a person standing in front of the
  machine, and the voice host speaks it through the speech pipeline where the
  deployment has a voice: the head is not moving, or its motion has stopped, or
  — where de-torquing could not be confirmed — that nobody should touch the
  head. Warnings are never spoken; each latch fires once per run, so a run
  speaks at most three sentences. A host built without a voice narrates and
  raises exactly as before, and every sentence a speaker refuses is an
  `unspoken` line on the console.

- **`reachy_host --check` compares the two names a speech run needs to agree.**
  The new `addressee` conclusion holds the host's own `pod` against the speech
  configuration's `[pods]` table: the pipeline addresses every motion script to
  the connected device's authenticated id, so a host answering to a name that
  is not one of them refuses every script it authors and its head never moves.
  A mismatch is an unheld conclusion, which `make speech-run` refuses on (exit
  11) before anything is provisioned.

- **Every payload build says which two brenn-pod revisions it is made of** — the
  revision `MODULE.bazel` pins for the crates the host links, and the one the
  brenn-pod checkout stands at, with the staged audio-device binary's age
  against that checkout's HEAD commit beside it, because the binary is whatever
  the last build there left behind and not the revision the checkout is on now.
  A note, never a refusal: a development checkout legitimately sits ahead of the
  pin, and a payload built out of two revisions otherwise fails on the unit as a
  handshake.

### Changed

- **The clip library asset carries motions, and sequence documents load
  again.** `ClipConfig` gained a derived `max_speed` — the highest invocation
  speed a clip's own frames admit, which a schedule's window is now screened
  against — and `ClipLibraryConfig` gained `motions`: one flattened motion per
  clip, plus one per composed sequence, each a lead gap and a list of
  (clip, speed, hold) segments. **A clip configuration emitted by an earlier
  build refuses to load**, loudly and typed rather than partially; regenerate
  it with `make clip-config`. The name sidecar gained a second table, keyed by
  motion id. **A schedule's overlay windows name motions**, not clips
  (`OverlayWindow.motion_id`), and an overlay plays a whole motion: its lead
  gap holds the base alone, its clips play end to end across their seams, a
  hold freezes the clip before it, and a channel one clip drives and the next
  does not fades out on the outgoing clip's ramp instead of vanishing. Since
  every clip is also a one-segment motion, naming a bare clip costs a schedule
  nothing.

- **Bazel is the only build system.** Every crate is a `rust_library` with its
  tests, `make check` is one lane (`bazel test --config=lint //...`) and CI one
  job, and the device binary is a cross-compile against the hermetic aarch64
  sysroot the pinned Clockwork drop brings — `--platforms=//bazel/platform:reachy-device`
  in place of cargo inside an emulated arm64 container. Nothing in the dev loop
  or on a runner invokes `cargo` or `rustup`, and the pinned `RUST_VERSION` in
  `MODULE.bazel` is the single compiler and the single statement of the edition.

- **rusty-cogs is consumed as it ships.** The two patches this repo applied to
  it — internal linkage for the generated signal trampolines, and dropping the
  root-only `include()` from its `MODULE.bazel` — are fixed upstream, so the pin
  carries no `patches` and `bazel/rusty-cogs-patches/` is gone. `rust_clk_module`
  now takes the repository word, the generation root and the crate name as
  parameters, so `bazel/rust_clk.bzl` is a thin wrapper over
  `@rusty_cogs//bazel:rust_clk.bzl` fixing this tree's repository word and
  naming policy instead of a verbatim copy re-synced by hand at every pin bump.
  With the copy go `bazel/BUILD.bazel`'s `framework_clk_imports` filegroup,
  which the macro now reaches for itself, and `cogs/upstream`'s longhand
  generator invocation, which is a macro call naming `repo` and `crate_name`.

- **A motion script this host authored and then refused is Critical, not a
  Warning.** A body offered to the host's gate now carries where it was authored
  — the pipeline's scripter and the motion harness are local, the bus is remote
  — and a local body the gate refuses means the head will not move for anything
  said to the robot until somebody edits a file. The alert says so once per run,
  under `reachy head refuses its own scripts`, and the refusal line on the
  console carries an `origin` field. A remote sender's refusal keeps the Warning
  it had: the intent channel is not assumed to carry one machine's traffic.

- **The speech-run analyzer opens the channel log and has an opinion about
  motion.** `speech_run_report` read the console alone and counted a dropped
  motion script as a note; it now reads the run's `.olog` beside it and fails a
  run whose scripts the host itself refused, whose session accepted none of what
  the pipeline authored, that the session never took the machine for, or whose
  head never measurably left its first pose. It also fails a run that handed an
  alert to a bus attachment that did not grant alerts, which loses it. A run
  nobody spoke to is still green. Runs that passed before this change can fail
  after it — that is the point of it.

- **The speech-run report prints one line per turn and writes an audio clip for
  each.** Every wake-word activation in a run is now a numbered turn whose line
  shows the transcript, the STT confidence scores (`no_speech`, `logprob`), the
  outcome (dispatched, declined, superseded), and the direction-of-arrival beam
  figure averaged over the turn's audio window. Each turn's raw audio is
  exported as a `.wav` file under `<run>.turns/`, carved from the frame-log
  store the run brought home, so a declined utterance can be listened to
  directly. The summary counts dispatched and declined turns and prints the
  `no_speech` range for each group, which is the reading that says whether audio
  quality degraded across a session.

- **Deploy fetches recorded audio alongside console logs.** `make speech-run`
  now brings back the frame-log recording store as `<run>.audio` after each
  speech run, and the preflight check validates that recording is configured
  with a relative store path and reports the per-device and per-pod storage
  caps. A configuration with an absolute recording path is refused.

- **`reachy_host --check` states the STT-confidence gate's thresholds.**
  The speech preflight conclusion now names the `no_speech` and `logprob`
  floors the gate will apply, so an operator can see what the pipeline will
  decline before a run starts. A configuration with no `[stt]` table says so
  rather than dropping the clause silently.

- **brenn-pod pin advanced to pick up the XVF3800 ASR-output routing and
  reliable queue lanes.** `BRENN_POD_REV` moves forward by two published
  commits: the cycle's speech-surface work (chip control registers, startup
  reboot, ASR-output channel routing, per-segment `base_sample`, and STT
  threshold narration) and the earlier reliable-lane queue rework. The host
  crates and the run report compile unchanged against the new surface.

- **A troubleshooting guide for speech degradation.** `docs/speech-degradation.md`
  explains the symptom (utterances declined after the first in a session), how
  to read a run's turn lines and `no_speech` scores, how to listen to a turn's
  clip, what the pod's chip state line says, and the two-session comparison that
  isolates the microphone board's adaptive processing as the cause.

- **Torn console lines are now recovered.** The run report's line classifier
  handles JSON with console text on either side — the shape every real
  transcript line arrives in when the host's console write and the pipeline's
  event write race on the same descriptor. Previously every such line was
  counted as noise and every transcript the tool had ever read was lost.

### Removed

- **The Cargo lane.** The workspace manifest, every crate manifest,
  `Cargo.lock`, `rust-toolchain.toml`, the `containers/bench-builder` image
  definition, and the `check-bazel`/`check-commit` split. Third-party versions
  are now stated once as `crate.spec` entries in `MODULE.bazel` and pinned by
  `MODULE.bazel.lock`. `make fix` formats only: `clippy --fix` has no Bazel
  equivalent, and clippy findings are fixed by hand from the gate's output.
  With the manifests go the ways cargo consumes these crates: there is nothing
  for a `git = "..."` dependency or a `cargo publish` to resolve. A consumer
  builds them with Bazel, or pins a revision from before this change.
