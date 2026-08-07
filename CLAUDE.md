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
- **A fault stops commanding and holds torque.** Cutting torque drops the head.
  Releasing is only ever an explicit operator action.
- **No automatic recovery.** No retry with perturbed inputs, no reboot, no
  auto-disarm. Recovery is a command.

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
health, resting pose — runs with no torque and no motion, and gates every
command that moves something.

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
