# brenn-reachy

Rust motion control for the Reachy Mini robot — kinematics, servo protocol, and
motion shaping, as libraries a real-time host can call.

## What this is

The Reachy Mini carries a six-legged parallel platform for the head, a rotating
base, and two antennae, all driven by Dynamixel X-series servos on one serial
bus. This repository is the control stack for that mechanism: the wire protocol,
the kinematics, the motion shaping, the bus layer, and a bench binary that drives
a real unit.

## Building it

Bazel, and only Bazel. `make check` is the whole gate — the shell scripts' own
self-checks, then `bazel test --config=lint //...` over every crate, every cog
and every lint aspect. There are no Cargo manifests and nothing here uses
`cargo` or `rustup`: the compiler, the third-party crates and the C++ sysroot all
come out of the module graph, so a fresh clone needs bazelisk and shellcheck and
nothing else. The device binary is a cross-compile of the same targets
(`make bench-build`; `docs/bench-runbook.md`).

An editor wanting a project model gets one from
`bazel run @rules_rust//tools/rust_analyzer:gen_rust_project`, which writes the
`rust-project.json` rust-analyzer reads instead of cargo metadata. No gate uses
it.

## The shape of the code

One constraint decides the whole design: **this code does not own the execution
loop.** A host application or a real-time middleware harness owns it, and calls
in. So:

- The libraries are sans-I/O. Kinematics, envelope checks, trajectories, the
  per-tick control step, and the arm/disarm sequences are pure functions over
  state the caller allocates and passes in. Nothing here holds a clock, a
  socket, a thread, or a hidden solver cache.
- Exactly one crate performs I/O, and it is the bottom of the stack: the serial
  bus. Everything above it speaks in joint angles and abstract requests.
- Large results are written into caller-provided output structures rather than
  returned by value, so a control loop can run without allocating.
- No async runtime. The bus is a blocking loop by design, because that is the
  shape every candidate host substrate wants at the wire.

## Why it is written defensively

The head hangs off a parallel linkage that has singular configurations a few
millimetres past the ends of its useful travel, and the per-leg crank limits are
the only thing holding it away from them. The servos apply a commanded position
as an immediate step with no trajectory of their own, so every gentle motion has
to be produced by the host. Both facts put the burden of not breaking the machine
on this code.

The response is uniform and deliberate:

- Every command path runs a positive envelope check before anything reaches the
  bus — per-leg travel windows, per-pose clearance from the singular
  configurations, yaw caps, antenna representability (finite, within the
  extended-position range).
- A violation is a typed error. It is never a clamp, never a saturated value,
  and never a quietly non-finite number.
- A fault stops commanding and holds position. It never silently continues, and
  it never releases the head, because releasing the head means dropping it.
- Hardware bring-up is done by writing a test that asserts the behaviour we
  expect and letting it fail. The failure output is the discovery; a confirmed
  reading is then baked into the test, which stays as a regression guard.

## Crates

| Crate | What it owns |
|---|---|
| `dxl-proto` | Dynamixel Protocol 2.0 frame codec, X-series register table, unit conversions. No I/O. |
| `reachy-kin` | Head kinematics for the parallel platform: inverse and forward solutions, travel envelope, clearance margins. Pure math. |
| `reachy-motion` | Trajectory shaping, the per-tick control step, and the arm/disarm sequences. Sans-I/O. |
| `reachy-bus` | The one I/O layer: serial port, transactions, error taxonomy, and the joint-to-servo map. |
| `reachy-bench` | Bench binary: a read-only self-test registry, and the supervised bring-up commands. |

The edges run one way: `reachy-kin` under `reachy-motion`, and both of those
plus `dxl-proto` under `reachy-bus`, with `reachy-bench` on top. `reachy-bus`
depends on `reachy-motion` because the joint-to-servo map is what joins the two
vocabularies — a joint and a register name on one side, an address and a count
on the other — and it is typed against both. The property that matters survives
the edge: `reachy-motion` still carries no I/O, no addresses and no counts.

### Clockwork vocabulary, Clockwork runtime

Clockwork **vocabulary** — the generated schema and enum crates, and
`clockwork_rs` (no dependencies, no I/O, no clock) — may be depended on
anywhere in this repo. Clockwork **runtime** — cog dials, channels, execution,
generated entry points, anything that makes code *run as a cog* — appears only
in `cogs/` compositions. The sans-I/O rule above is unchanged and still binding:
a library takes time as a parameter, owns no ports, no sockets, no sleeps, and
reads no clock. State that has to survive between executions lives in a schema,
and a library reaches it the same way a cog does — by reference.

Generated types come in two surfaces. The validated one carries the plain name
(`FaultKind` is a real Rust enum, `CommissionSnap` a struct of plain fields over
the slot's own bytes) and is what logic code speaks; the open one carries a
`Wire` suffix and appears only at a wire boundary, where one
`validate()`/`validate_mut()`/`clear_valid()` call per crossing narrows it and a
failure is a typed refusal. Every module in the tree generates under that
naming; a consumer still reading a slot through the open surface writes the
`Wire` type and is ported to the validated view as its layer is reworked.

A `.clk` module lives in a top-level package named for the component that writes
or publishes its types — `motion/` for what `reachy-motion` writes, `driver/` for
what the motor driver publishes and consumes, `cogs/` for what a cog writes —
and consumers import from there. `geometry/` is the one
exception, holding the mathematical carriage types no single component produces;
its name is the whole licence for what may go in it. There is no central schema
bucket: a path has to say what a module is about before a reader opens it. The
directories are not the hyphenated crate directories because a module's `use`
path and generated crate name are its path from the repo root verbatim.

Within a package the knife cuts to minimise visibility, because visibility is
recompilation: one `.clk` module per type or per tightly coupled set, so that a
change to a sequencer's state does not rebuild the clip player.

## Status

Implemented through the bench milestone: all five crates are functional and
tested, and the read-only self-test registry has run green against a real unit.

Supervised motion bring-up is in progress. The arming sequence has enabled
torque on hardware, and no commanded motion has run yet.

## Attribution

Substantial facts and shaping in this implementation derive from published
open-source material — notably the Apache-2.0 `reachy_mini` SDK and its
description files, the Apache-2.0 `rustypot` crate, and the servo vendor's public
documentation. Attribution for adapted material is recorded in `NOTICE` as it
lands. The code itself is a fresh Rust implementation.

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
