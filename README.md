# brenn-reachy

Rust motion control for the Reachy Mini robot — kinematics, servo protocol, and
motion shaping, as libraries a real-time host can call.

## What this is

The Reachy Mini carries a six-legged parallel platform for the head, a rotating
base, and two antennae, all driven by Dynamixel X-series servos on one serial
bus. This repository is the control stack for that mechanism: the wire protocol,
the kinematics, the motion shaping, the bus layer, and a bench binary that drives
a real unit.

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
  configurations, yaw caps, antenna range.
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
| `reachy-bus` | The one I/O layer: serial port, transactions, error taxonomy. |
| `reachy-bench` | Bench binary: a read-only self-test registry, and the supervised bring-up commands. |

## Status

Early. The repository is scaffolded, licensed, and gated; the crates above are
declared but not yet implemented. Nothing here has driven a motor.

## Attribution

Substantial facts and shaping in this implementation derive from published
open-source material — notably the Apache-2.0 `reachy_mini` SDK and its
description files, the Apache-2.0 `rustypot` crate, and the servo vendor's public
documentation. Attribution for adapted material is recorded in `NOTICE` as it
lands. The code itself is a fresh Rust implementation.

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
