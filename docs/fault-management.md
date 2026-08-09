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

## MRMs: the two Minimum Risk Maneuvers

Two maneuvers reach the MRC. Which one applies is decided by whether motor
control is currently trusted.

- **MRM-A (controlled):** command a stow move, then de-torque the motors.
  Used whenever control and feedback are healthy: orderly shutdown, script
  timeout, loss of instruction, the end of an interaction — every
  *expected* ending.
- **MRM-B (uncontrolled):** write torque-off to every servo, immediately,
  best-effort, and let the head fall into approximate stow under gearbox
  resistance. Used whenever motor control or position feedback is degraded or
  untrusted — which is what a fault *means* — and as the fallback when MRM-A
  itself fails partway.

MRM-B is not an emergency compromise to be minimized; on this platform it is a
sanctioned, adequate maneuver. A head that flops gently into near-stow is at
the MRC.

## Unattended operation is normal

Nobody watches this machine work. Resting at the MRC and arming from it on a
wake word — torque-on with no operator present — is sanctioned normal
operation, not a special mode; so is releasing back to the MRC unattended,
whether by MRM-A at an expected ending or MRM-B on a fault. The one act
reserved for an operator is restarting a process that has parked after a
fault. This closes the auto-arm question the service-mode work deferred:
boot, and every wake after it, may arm.

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
- **Commanding rules are unchanged and are not fault management.** The
  envelope check on every commanded target, typed errors instead of clamps,
  and per-tick step bounds all remain binding — they gate what we *ask* the
  machine to do. None of them may ever gate a torque-off.

## Fault classes and their responses

| class | examples | response |
|---|---|---|
| Expected ending | shutdown signal, script timeout, end of interaction, bridge gone | MRM-A: stow, then de-torque |
| Expected refusal | rail below the voltage floor at arm time, servo hardware-error bit set before torque-on | Don't arm; report; retry on the next request. No torque was written, so there is nothing to maneuver |
| Fault | tracking loss, step-bound violation, read loss, hardware error bits mid-run, bus errors, a failed MRM-A | MRM-B: immediate best-effort torque-off, then report and stop commanding |

After a fault, the process stops commanding and stays parked **at the MRC**
until an operator restarts it. The fault is never auto-cleared and never
retried with perturbed inputs — recovery (re-arming after a fault) is a
command. What is autonomous is reaching the MRC itself.

## History, briefly

The pre-2026-08-08 stack did the opposite of this doctrine: it parked faults
with torque held, refused to disarm away from verified stow, and refused to
arm over a 0.5° position settle (the first hardware arm measured 0.615° of
ordinary torque-on settle and was refused for it). None of that was a project
requirement; it accreted as reflexive caution. The 2026-08-08 post-mortem
replaced it with this document.

## Scope

This doctrine is for **this robot**. The hazard analysis at the top is what
licenses everything under it. A different platform — heavier, stronger,
pinch-prone, or load-bearing — gets its own hazard analysis first; nothing
here transfers by default in either direction.
