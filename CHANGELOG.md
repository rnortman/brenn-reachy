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
