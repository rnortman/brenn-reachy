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
