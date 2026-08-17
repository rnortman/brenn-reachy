# brenn-reachy

Rust motion control for the Reachy Mini: servo protocol, head kinematics, motion
shaping, bus layer, bench binary. Charter and the architectural constraint:
`README.md`. Read it first — the sans-I/O rule and the no-clamp rule there are
binding on every change in this repo.

## Safety rules that are not negotiable

- **Every command path runs the envelope check first.** Reachability, per-leg
  travel windows, per-pose clearance from the linkage's singular configurations,
  yaw caps, antenna representability (finite, within the extended-position
  range).
- **A violation is a typed error.** Never a clamp, never saturation, never a
  non-finite number handed onward. If you find yourself reaching for
  `clamp`/`min`/`max` on a commanded value, stop.
- **Fault management follows `docs/fault-management.md`.** Read it before
  touching anything that arms, disarms, or handles a fault. The short form:
  the Minimum Risk Condition is *stowed and de-torqued*; a fault response
  de-torques the motors it covers (a controlled stow first when control is
  trusted, an immediate best-effort torque-off when it is not); **nothing ever
  gates de-torquing**, and holding torque is never a fault response — stowed
  with torque held is this machine's only pinch hazard. "The motors it covers"
  is load-bearing: a response may be scoped to one group, and an antenna pair
  going limp while the head keeps its presence is a fault answered, not an
  exception to this rule.
- **No automatic fault recovery.** Nothing clears a fault, and nothing retries
  a failed operation with perturbed inputs. The park-class responses wait for
  an operator to restart the process. The rest-class ones end the session
  instead: the machine is at the Minimum Risk Condition and the next wake
  builds a fresh session, which is a new engagement rather than a recovery of
  the one that stopped. Reaching the MRC itself is always autonomous.

## Gates

A commit scans the staged change for secrets and then runs `make check`; a push
scans the range being pushed. Both hooks are wired by `make setup-hooks`, once
per clone. CI independently scans the tree and runs `make check` on every push
and pull request.

Treat the local hooks as the gate and CI as a backstop, not the other way round.
A CI run says nothing about the commit sitting in your working tree.

If a gate blocks a write or a commit, **surface it** — never route around it. A
gate being wrong happens and is worth reporting; a bypassed gate is not
recoverable once it has been pushed.

Every manifest carries `license.workspace = true` against the workspace's
`license = "Apache-2.0"`. Without it the published code is technically
all-rights-reserved.

## Bring-up discipline

We bring up hardware — and untried features of hardware we already use — by
writing a self-test that **asserts the behaviour we expect and letting it fail**,
not by writing throwaway probe code. The failure output is the discovery.

Once an observed value is confirmed correct-and-expected, bake it into the test.
It stays in the bench self-test registry permanently as a regression guard, so
an expensive hardware round-trip yields a durable asset instead of a discarded
script.

Guardrail: an **unexpected** reading gets human review *before* you make the
test pass. Do not let make-it-green launder an unexpected value into accepted
truth. Keep presence tests (does the servo at this ID answer) separate from
identity tests (does this register read this value).

The read-only half of the registry — presence, register sweeps, voltage,
health, resting pose — runs with no torque and no motion. The registry is a
diagnostic and a regression guard; it does **not** gate arming or commanding.
Gating routine operation on self-test records was bring-up caution, retired by
`docs/fault-management.md`.

Getting the bench onto a real unit — the build and deploy path, which services
hold the servo bus, the trace workflow, the soak, and the anomalies nobody has
explained yet — is `docs/bench-runbook.md`.

## Device deployment doctrine (dev cycles)

During development iteration, **nothing we push touches the device's eMMC**.
Binaries, configs, tokens, secrets — all of it lands in RAM (tmpfs) and is
re-pushed after a reboot by one command. This is the brenn-os design: the only
flash-resident state is fundamental remote-access credentials and identity.
Flash-backed ("baked") placement of anything else is a release-hardening act
performed on a stable release — never a dev-cycle convenience. Do not propose
persisting app state to `/persistent` (or anywhere else on flash) to make a
dev workflow nicer; fix the deploy command instead.

## Comments

Comments say what the code currently does and why, tersely. Two standing bans:

- **No references to design or decision documents.** Those are ephemeral and
  live outside this tree; a comment citing one rots when the document moves. The
  code must stand on its own. The secret scanner enforces the common shape of
  this on diffs.
- **No changelog comments.** What the code used to do, or how it changed, is
  what `git log` and `CHANGELOG.md` are for.

Dependency lines in every manifest carry a comment saying what the dependency is
for. A `//!` header on every module states what it is and why it exists.

## TODO system

Two pieces that stay in sync:

- `TODO.md` at the repo root — the master list. Each entry has a slug, a
  description, and the deferral context.
- `TODO(slug)` comments in code, marking the spot where the work happens.

Slugs are the join key; adding a TODO requires both halves. Don't use TODOs for
vague aspirations — every TODO describes a concrete thing, in a place where
"done" is obvious. `TODO.md` is for code and design work only, never for
operational tasks; those are asks made to a human directly.

`TODO.md` is part of a public repository. An entry is public writing: it may
describe the work, but not internal topology, host names, or anything else that
would not otherwise be published.

## Comments in `.clk` files

In `.clk` files, `//` is not a normal code comment but a *required documentation
element*. They cannot be removed or relocated. They may be rewritten.
