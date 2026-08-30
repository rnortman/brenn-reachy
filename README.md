# brenn-reachy

Rust motion control for the Reachy Mini robot — kinematics, servo protocol, and
motion shaping, as libraries a real-time host can call.

## What this is

The Reachy Mini carries a six-legged parallel platform for the head, a rotating
base, and two antennae, all driven by Dynamixel X-series servos on one serial
bus. This repository is the control stack for that mechanism: the wire protocol,
the kinematics, the motion shaping, the bus layer, the Clockwork cog system that
hosts them, and a bench binary that talks to a real unit's hardware.

## Building it

Bazel, and only Bazel. `make check` is the whole gate — the shell scripts' own
self-checks, then `bazel test --config=lint //...` over every crate, every cog
and every lint aspect. Nothing here uses `cargo` or `rustup`: the compiler, the
third-party crates and the C++ sysroot all come out of the module graph, so a
fresh clone needs bazelisk and shellcheck and nothing else. One Cargo manifest
exists, `crates/motion-proto/Cargo.toml`, and it builds nothing here: it is the
export surface for a Cargo workspace elsewhere that pins that crate by revision,
held to the crate's `BUILD.bazel` by a gate test. The device binary is a
cross-compile of the same targets
(`make bench-build`; `docs/bench-runbook.md`).

The voice host's dependencies arrive the same way and are worth naming, because
they are the only ones that are not plain Rust from crates.io. The pipeline
libraries come from the brenn-pod repository as git pins at revisions published
on its public remote — a pin at an unpublished revision resolves for nobody, so
it is not a pin. Two native libraries come with them: OpenSSL, from the module
graph's own pinned build, and ONNX Runtime, from a hash-pinned release archive.
Both crates' discovery build scripts are switched off, because what those
scripts do is search the machine and, for ONNX Runtime, download; what the
answers should be is stated in `MODULE.bazel` and checked against the linked
libraries by `//crates/reachy-host:host_closure_test`.

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
- No async runtime on the control path. The bus is a blocking loop by design,
  because that is the shape every candidate host substrate wants at the wire,
  and the motion and control stack holds no runtime at all. The voice host
  binary is the one exception and is quarantined off that path: it hosts tokio
  for the network edges it owns, exactly as the driver hosts a serial port.

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
| `motion-proto` | The motion intent wire contract: the timed script a speech interaction publishes, its decode tolerance and validation refusals, and the publisher's sequence source. No I/O, no clock. |
| `reachy-kin` | Head kinematics for the parallel platform: inverse and forward solutions, travel envelope, clearance margins. Pure math. |
| `reachy-motion` | Trajectory shaping, the per-tick control step, and the arm/disarm sequences. Sans-I/O. |
| `reachy-clips` | Recorded motions as masked per-channel deltas, and how they layer over live motion. Sans-I/O. |
| `reachy-driver` | A motor driver's decisions with no motors attached: the goal gate and its dead-man, the auxiliary-transaction slot, the torque-off confirmation. Sans-I/O. |
| `reachy-bus` | The one I/O layer: serial port, transactions, error taxonomy, and the joint-to-servo map. |
| `reachy-bench` | Bench binary: a read-only self-test registry and the bare-bus commands. It moves nothing. |
| `reachy-edge` | The intent edge: a motion script from outside, screened — size, addressee, redelivery — and compiled into the request the session screens. Sans-I/O. |
| `reachy-ask` | The motion harness's intent source: one pinned gesture through the real intent edge, and the session's narration rendered as JSON lines. |
| `reachy-host` | The voice host process: the configuration it is built with, the running gate over `reachy-edge`, and the queue both intent sources hand bodies to. |
| `reachy-motord` | The driver process: a 20 ms grid on the real clock, the serial port, and `reachy-driver`'s decisions, meeting the cogs over UDP. |
| `cogs/` | The Clockwork compositions that host the libraries: the `Mover`, `Pose`, `Session` and `MotorSim` cogs, their schemas, the deterministic scenario suite, the online composition and its intent-edge sockets, and the log analyzer. |

The edges run one way: `reachy-kin` under `reachy-motion`, and both of those
plus `dxl-proto` under `reachy-bus`, with `reachy-bench`, `reachy-motord` and
the cog compositions on top. `reachy-bus`
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

The read-only self-test registry has run green against a real unit, and the
arming sequence has enabled torque on hardware. No commanded motion has run on
hardware yet.

Motion lives in the cog system. The four motion crates run as the `Mover`,
`Pose`, `Session` and `MotorSim` cogs against a simulated plant under the
deterministic scenario runner, which is where the whole engage-to-rest lifecycle
— commission, engage, tracking and health faults, the fault ladder's stow and
de-torque responses — is pinned. The bench binary's own motion layer, which was
the host for supervised bring-up, is deleted: the bench validates that we can
talk to the hardware and nothing else.

The host that puts the cog system on a real bus is built and has run on one:
`reachy00` completes runs end to end, and every verdict so far is red for
reasons the runbook's open observations record. `reachy-motord` is the driver
process — a 20 ms grid on the real clock, the serial port, and
`reachy-driver`'s decisions — and it meets the cogs
over six UDP sockets on loopback, filling the slots the simulated plant fills in
the scenario suite. The online composition is that seam plus the control core, a
logger process, and two loopback sockets where a producer of scripts stands.
`make check-device` cross-compiles everything built for a unit — the payload's
three binaries and the composition they are staged from, plus the bench binary,
named once as `//bazel/platform:device_deployables` — then
disassembles the C++ two and refuses any instruction the unit's Cortex-A72 does
not implement, and CI blocks on both halves;
`docs/bench-runbook.md` is the procedure for the first run, and
`//cogs:first_motion_report` is what reads its log.

## Attribution

Substantial facts and shaping in this implementation derive from published
open-source material — notably the Apache-2.0 `reachy_mini` SDK and its
description files, the Apache-2.0 `rustypot` crate, and the servo vendor's public
documentation. Attribution for adapted material is recorded in `NOTICE` as it
lands. The code itself is a fresh Rust implementation.

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
