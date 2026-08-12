# Fault Management — Reachy Mini motion

This document is the standing fault-management doctrine for the Reachy Mini
motion stack: `reachy-motion`, `reachy-bench`, `reachy-bus` in this repo, and
`reachy-motiond` in brenn-pod. It binds every change that touches arming,
disarming, fault handling, or anything that gates a torque transition.
`CLAUDE.md` in both repos points here so the argument does not get re-litigated
one gate at a time.

## Ground truth: what this machine can actually hurt

The Reachy Mini is very nearly harmless. It is small, light, and mostly
plastic; the motors are weak; it has no arms and almost no pinch points. The
servo gearboxes provide enough resistance that motion under gravity is
self-limiting: de-power the motors with the head raised and the head settles
into something approximating the stow position with only a little violence,
because it never has far to fall and cannot fall fast. The risk of damage is
mostly to the robot itself, not to any person or thing near it.

The largest risk this device poses to its surroundings is the antennas
sweeping through their outboard (sideways) arc and knocking something over —
which is why the antenna arc policy exists in the motion code, and it is a
*commanding* concern, not a fault-response concern.

Consequence for everything below: fault handling on this platform optimizes
for simplicity and for getting to the safe condition fast. It does not import
industrial-robot caution the hazards cannot justify.

## MRC: the Minimum Risk Condition

**Head stowed, motors unpowered.**

The antennas are minimum-risk in almost any position — they are light enough
that with torque off they stay wherever they are. Down is nicer; it is not
required for the condition to count as reached.

A machine at the MRC needs nothing from software and threatens nothing. It is
the correct resting state, the correct post-fault state, and the correct
state to reach on shutdown, loss of instruction, or any other end of an
activity.

## The maneuvers: how the MRC is reached

Four maneuvers, each with a slug. The slug is the name the maneuver is
reported under everywhere — timeline entry, operator line, daemon alert — so
a maneuver that happened can be named rather than described.

| maneuver | what happens |
|---|---|
| `slow_stow` | abandon the current move — the offending sample is never emitted — and stow every commanded joint under control, full checks live, then release |
| `masked_slow_stow` | torque off the servo that dropped out (an explicit torque-off write: masked is de-torqued, never merely uncommanded), never command it again, and stow on what still commands; a further servo dropping out expands the mask and the stow carries on |
| `antenna_torque_off` | write torque-off to the two antenna servos and stop commanding them; the head is untouched and its move carries on |
| `immediate_all_torque_off` | immediate best-effort torque-off of all nine, letting the head settle into approximate stow under gearbox resistance |

**The selection rule: a controlled stow whenever control is trusted.** A hand
pushing the head down is met by a stow that yields with it, not by a machine
going limp; a servo that dropped out is met by a stow on the five that still
answer. `immediate_all_torque_off` is for the faults that say control itself
cannot be trusted — no feedback, no believable pose, no bus — and for
fall-through when a controlled maneuver is defeated.

`immediate_all_torque_off` is not an emergency compromise to be minimized; on
this platform it is a sanctioned, adequate maneuver. A head that flops gently
into near-stow is at the MRC.

## Unattended operation is normal

Nobody watches this machine work. Resting at the MRC and arming from it on a
wake word — torque-on with no operator present — is sanctioned normal
operation, not a special mode; so is releasing back to the MRC unattended,
whether by a controlled stow at an expected ending or by an immediate
torque-off on a fault. The one act reserved for an operator is restarting a
process that has parked after a fault. This closes the auto-arm question the
service-mode work deferred: boot, and every wake after it, may arm.

## The forbidden response

**Never hold torque as a fault response.** Not frozen in place, not stowed.

If this device has any pinch hazard at all, it is motors held torqued at the
stow position — that is the one configuration where the machine could actively
squeeze something. Freeze-with-torque-held is therefore the *only* fault
response that is actually bad. Anything else is better. It was never a
requirement of this project; it must not reappear.

## Gate policy

- **Zero gates on de-torque.** Nothing — no position check, no verify step, no
  state precondition — may ever refuse, defer, or condition writing torque-off.
  De-torquing the motors *is* the safety action; a gate in front of it is a
  gate in front of safety. A stow-verify before an orderly release is fine as
  a *reported outcome*; it must never decide whether torque comes off.
- **Torque-on gates are allowed, minimal, and enumerated.** Refusing to *arm*
  costs nothing and can be legitimate — but only for conditions that make
  powering the motors themselves unwise (a sagging supply rail, a servo
  reporting a hardware error). The current position of the machine is never
  such a condition: if the motors are off, the head is at or near stow, however
  crookedly it got there. Measure where the joints actually are, power on
  (these servos hold position at torque-on; there is no jerk), and plan a move
  from the measured pose to the target. "The machine is not standing where I
  expected" is a reason to measure, not to refuse.
  A hardware-error bit refuses the arm only when it is on a servo that carries
  the head; bits on an antenna alone engage the head and leave that antenna
  limp for the session, because one latched antenna overload must not make the
  head un-armable until somebody power-cycles it.
- **Commanding rules are unchanged and are not fault management.** The
  envelope check on every commanded target, typed errors instead of clamps,
  and per-tick step bounds all remain binding — they gate what we *ask* the
  machine to do. None of them may ever gate a torque-off.
- **A step bound bounds the plan, and only the plan.** The loop advances a
  move's own clock by at most one nominal period however late it wakes, so a
  period that starts a second late resumes the same path a second later
  instead of commanding a second's worth of travel in one step. Scheduler
  lateness therefore cannot push a healthy move past a bound; a bound that is
  crossed is an interpolator or a seed that is wrong. That is what makes it
  safe to size the bounds against the plan's own measured peak and nothing on
  top of it.

## Faults and the responses that answer them

**The classification criterion: a faulted motor is one we cannot command
anymore.** Everything else is either a software defect (the machine is fine;
our ask was bad) or a degradation confined to one joint group. A software
defect is not a fault and is never answered by parking the robot.

**The responses.** A response is a maneuver plus the state it leaves behind.
Six, and nothing outside them:

| response | maneuver | post-state | latches? |
|---|---|---|---|
| `refuse` | none; the ask is declined | unchanged | no |
| `slow_stow_to_rest` | `slow_stow` | Resting; the next wake re-engages | no |
| `degrade_antennas` | `antenna_torque_off` | the session continues with the antennas out of service until the next engage retries them | per session |
| `immediate_all_torque_off_to_rest` | `immediate_all_torque_off` | Resting; the next wake re-engages | no (alert every occurrence) |
| `masked_slow_stow_to_park` | `masked_slow_stow`, then unconditional torque-off of all nine | Parked until an operator restarts the process | yes |
| `immediate_all_torque_off_to_park` | `immediate_all_torque_off` | Parked until an operator restarts the process | yes |

**The faults.** These are the only conditions that are faults. One vocabulary
for the whole stack; the tick raises the first six, and the layer holding the
bus raises the last two, which are verdicts about transactions rather than
about anything a control step can see.

| slug | what it detects | response |
|---|---|---|
| `antenna_obstructed` | an antenna past the tracking threshold for a whole window without closing: interference, a snag, a hand | `degrade_antennas` |
| `antenna_servo_fault` | hardware-error bits on an antenna servo, mid-run or at engage | `degrade_antennas` |
| `head_obstructed` | a leg or the body yaw past the threshold for a whole window without closing: a grab, a snag, a jam. Not a motor failure — the servo still commands | `slow_stow_to_rest` |
| `head_servo_fault` | hardware-error bits on a leg or body-yaw servo mid-run | `masked_slow_stow_to_park` |
| `position_feedback_lost` | too many consecutive periods with no usable position read; a reading nobody can place counts as one of them | `immediate_all_torque_off_to_park` |
| `measured_pose_invalid` | the measured cranks yield no believable head pose for a whole run of live reads — a mechanism outside its own model | `immediate_all_torque_off_to_park` |
| `bus_failure` | transactions failing under torque; a write nothing acknowledges after every attempt | `immediate_all_torque_off_to_park` |
| `torque_off_unconfirmed` | a torque-off write unacknowledged after all nine attempts and their retries | `immediate_all_torque_off_to_park`, degenerate: the torque-off already ran, so what remains is the park and the alert. An unconfirmed MRC is never reported as Resting |

Classification happens exactly once, at the point the condition becomes one of
these values, and travels as that value. No layer re-derives a class from a
message, and no layer invents a response outside the table.

**What is not a fault.** A goal that steps further in one period than the
bound allows, a sampled path that leaves the envelope, our own clock or budget
running out, a malformed configuration, a command the envelope refuses. These
say the plan was wrong, not the platform. The move is abandoned where it
stands, the offending sample is never emitted, and the machine — healthy, and
still taking goals — winds down under control. None of them park anything.

## The escalation ladder

A fault raised *inside* a controlled wind-down never starts a second answer to
the same incident. It does exactly one of two things:

- **A fault whose response is already a per-servo torque-off expands the
  running maneuver.** A servo dropping out mid-stow, or an antenna failing
  mid-stow, is torqued off on the spot, dropped from every further command and
  check, and the stow carries on with what remains. The mask only grows, and
  each growth *is* a torque-off write rather than a gate on one, so sequential
  single-servo failures walk the head down instead of dropping it. When the
  mask covers every joint that carries the head, the maneuver completes as the
  full torque-off on the spot — and is reported as unconfirmed, because
  nothing watched the head land.
- **Everything else falls through to `immediate_all_torque_off`.** A head
  obstruction names no motor to mask — on a six-crank parallel head a single
  crank stopping cannot be blamed on that servo rather than on the platform,
  and masking it would grind the stow against the obstruction where yielding
  is the point. The control-not-trusted faults fall through for the reason
  they exist.

**One clock.** The stow clock and its budget start once and run through every
expansion; nothing restarts them. Expiry, budget exhaustion, or a
fall-through-class failure ends the maneuver at `immediate_all_torque_off`
regardless. Mask expansion therefore strengthens the termination guarantee
rather than weakening it — nothing in the ladder gates a de-torque.

**Disposition is the sticky maximum.** A fall-through keeps the wind-down's
own disposition; a servo dropping out during a stow that was heading for rest
upgrades the ending to park, because that fault latches. Antenna expansion
never changes the disposition.

## Latching, and what recovery means

A **park**-class response latches: the process stops commanding and stays at
the MRC until an operator restarts it. That fault is never auto-cleared and
never retried with perturbed inputs.

The rest-class responses (`slow_stow_to_rest`,
`immediate_all_torque_off_to_rest`) do **not** latch. They are endings, not
verdicts about the machine: the session is over, the machine is at the MRC,
and the next wake engages a fresh session normally. Nothing resumes the
session that stopped — recovery is always a new engagement or a person, never
a cleared flag on the old state.

`degrade_antennas` latches for the session only: the antennas stay out of
service until the next engagement, which retries them and takes them back if
their bits have cleared. There is no in-session clear.

What is autonomous is reaching the MRC itself.

## How a fault is reported

One channel: an append-only, per-session **fault timeline** of typed entries —
a fault with its slug and detail, a response with its maneuver and how far it
got (started, expanded, fell through, ended unconfirmed, completed). A
compound ending reads back as the sequence it was, so "the head was grabbed,
the stow started, a servo dropped out, the stow carried on without it,
everything ended limp" is data rather than prose to be reconstructed.

Readable two ways from the start: **pull** — the record is handed out with the
result and queryable while the session runs; **push** — a subscriber receives
each entry as it appends, which is how a daemon turns a fault into an alert
without polling. An operator line is a *rendering* of entries. Nothing parses
a class back out of rendered text.

The timeline never grows at poll rate: a fault appends the once, on the period
it is raised, so a servo whose error bits stay latched for the rest of the
session adds nothing after the entry that took it out of service.

## Attended and unattended differ in exactly one place

The post-state column above is the unattended (daemon) behavior. The attended
bench diverges only for the endings that name **no** condition of the machine
— a refusal, and the software-defect triggers of `slow_stow_to_rest`. Those
report and leave the machine holding: an operator is standing next to it, `off`
is one command away, and the bench does not tick between commands, so a held
pose cannot re-raise anything.

A *fault*-triggered `slow_stow_to_rest` runs the stow on the bench too, and
then de-torques. Holding torque against a hand or a snag would be a fault
answered by holding — the forbidden response — and yielding under control is
the whole point of the maneuver, attended or not. Everything from
`degrade_antennas` down is identical on both.

## History, briefly

The pre-2026-08-08 stack did the opposite of this doctrine: it parked faults
with torque held, refused to disarm away from verified stow, and refused to
arm over a 0.5° position settle (the first hardware arm measured 0.615° of
ordinary torque-on settle and was refused for it). None of that was a project
requirement; it accreted as reflexive caution. The 2026-08-08 post-mortem
replaced it with this document.

The classification above replaced a second accretion: a blacklist in which
every software surprise was treated as an untrusted machine, so a planner
defect and a grabbed head both dropped the head and parked the daemon. The
enumerated faults, the maneuver slugs and the escalation ladder exist because
the response set has to be small enough to hold in the head and complete
enough that nothing needs a default.

## Scope

This doctrine is for **this robot**. The hazard analysis at the top is what
licenses everything under it. A different platform — heavier, stronger,
pinch-prone, or load-bearing — gets its own hazard analysis first; nothing
here transfers by default in either direction.
