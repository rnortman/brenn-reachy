# TODOs

Entries are slugs joined to `TODO(slug)` comments in the tree. See `CLAUDE.md`
for the convention — including that this file ships publicly.

## `example-placeholder` (DO NOT TRIAGE — this is a fake entry)

This is a placeholder entry. Leave it here so the file is never empty. It is not
a real TODO. You would reference it in code with a `TODO(example-placeholder)`
comment. That is the whole design: an entry here with a slug, joined to code
comments by that slug. Add real TODOs below this one, in this format.

## `bus-watchdog-policy`

Decide whether the servos' Bus Watchdog register is armed, and with what
timeout. It stays at its factory-disabled value of 0 for now.

Deferral context: the watchdog stops a servo holding its goal when the host goes
quiet, which on this linkage means the head falls, and a latched watchdog then
answers writes with the same Data Range error an out-of-range goal produces —
two failures that would be indistinguishable at the bus layer. Arming it is only
worth considering once the command loop's real timing has been measured on
hardware, so the timeout can be set from data rather than guessed. Marked at the
register definition in `crates/dxl-proto/src/regs.rs`.

## `clip-doc-faithful-blends`

Let a written clip document state the ramps it actually plays. `Clip::to_doc`
omits any blend longer than the clip itself, because a stated ramp that long is
refused at load, so the reload re-derives it instead of reading it.

Deferral context: the omission works — the reload lands on the same clip under
the same limits, pinned by
`a_clip_whose_derived_ramps_outrun_it_survives_a_round_trip` — but it costs
three things: the file no longer says what plays for exactly the clips whose
ramps were unusual, a reader with different step bounds derives a different ramp
with nothing in the file to compare against, and every reload of such a clip
reports a `BlendStretched` correction against a file that stated nothing. The
fixes are both design decisions rather than code ones: exempting a stated ramp
that equals the derived floor is the load-time interaction between authored and
derived numbers that the ceiling's design explicitly ruled out, and a separate
document field for a derived ramp is a format change. Marked at `Clip::to_doc`
in `crates/reachy-clips/src/format.rs`.

## `collision-envelope`

Bound the linkage against itself. Nothing in the envelope check currently does:
it covers reach, per-leg travel, clearance from the singular configurations,
yaw, head attitude and antenna range, and none of those notice a rod touching
another rod.

Deferral context: in a band of head heights roughly 13 mm below nominal and
15 mm above the bottom of travel, the crank travel windows stop binding on
head-relative yaw entirely, and what limits it there is rod-to-rod interference
— a separation that falls to a few hundredths of a millimetre at large relative
yaw and to zero at half a turn. The working relative-yaw cap keeps commanded
poses far outside that regime, so nothing in the milestone approaches it, and
the check that would replace the cap needs the collision geometry the vendor
publishes at three fidelities plus a segment-distance test that the envelope
does not currently carry. Marked at `EnvelopeConfig` in
`crates/reachy-kin/src/envelope.rs`.

## `geometry-fit`

Measure this unit's crank length, rod length, base radius and platform radius on
the bench, and substitute the fitted values for the vendor's nominal ones in
`HeadGeometry`.

Deferral context: the defaults are the vendor's published nominal model, and a
second parameter set differing from it by a few millimetres has been written down
for this linkage — evidence that more than one set exists, not that more than one
build does. Millimetres are large against the clearance the crank stops leave at
the top of vertical travel, which is about one, so the nominal numbers should not
be treated as the only ones that have ever described the mechanism. The struct is
already a runtime configuration seam for exactly this substitution; what is
missing is the measurement, which needs the machine. Marked at `HeadGeometry` in
`crates/reachy-kin/src/geometry.rs`.

## `health-read-budget`

Decide whether a run of health sweeps that fall short should stop the tick loop
the way a run of missed position reads does, and if so after how many.

Deferral context: the tick's per-period position read has a miss budget behind
it and ends in a typed fault; the one-hertz health sweep has neither, so a servo
that answers its position cleanly and refuses the hardware-error register leaves
a move running with no health verdict at all. Every such sweep is now reported to
the operator as it happens and counted in the move's summary, which is what a
supervised run needs to see it. What is not decided is whether it should also
*stop* the move: the sweep is the only detection of a latched overload or
overheat, but a fault here holds torque and ends a session, and how often a
sweep really falls short on this bus is unmeasured — the first supervised runs
with torque on are what say whether the answer is a budget, a config key beside
`read_loss_ticks`, or a documented decision that a health gap never faults.
Marked at `read_health` in `crates/reachy-bench/src/pump.rs`.

## `provisioning-repair`

Decide whether this project ever repairs a servo's *vendor*-provisioned setup
registers — homing offsets, travel limits, current limits — and behind what
evidence. Today it verifies them and repairs none of them.

Deferral context: those registers are non-volatile, and a servo silently ignores
a write to one while its torque is on, so a repair that appeared to succeed
could have changed nothing — the worst available outcome. The guarded path that
answers that now exists, scoped to the one register this project provisions
itself: it reads Torque Enable, refuses unless it is off, writes, and reads back
count-exact, and the `provision` command is its only caller. Every other write
path still refuses a non-volatile register outright. What is still undecided is
the vendor's half: a unit that arrives part-provisioned is a typed refusal
naming the servo and the register, with the vendor's own setup tool as the fix,
and that remains the right answer while one machine is in play. What would
change it is a second unit, or a servo replaced in the field, at which point the
question is which registers this project is willing to author rather than
compare. Marked at the non-volatile refusal in `crates/reachy-bus/src/bus.rs`.

## `rail-curve`

Set the supply floor arming refuses to proceed below from a measurement of what
the rail actually does under load, and decide whether one threshold covers both
a bench supply and a battery.

Deferral context: the figure in the code is 6.0 V — a round number above the
servos' own minimum-voltage alarm and below anything a healthy supply should sag
to, chosen with a margin rather than measured. What matters is the sag while nine
servos take up the head's weight, which is the moment arming enables torque, and
nobody has recorded the rail through that transient on this platform. Too high a
floor refuses to arm a machine that would have been fine; too low a floor arms
one that will brown out mid-motion, and a brown-out under load drops the head.
The reading needs the machine and a scope or a logging meter on the rail. Marked
at `DEFAULT_MIN_ARM_VOLTAGE` in `crates/reachy-motion/src/arm.rs`.

## `unsendable-frame-condition`

Decide which condition of the machine a frame this host could not send names,
when it happens with torque on — and whether that is `bus_failure`, a condition
of its own, or none at all.

Deferral context: `wire_failure` collapses the six transaction failures where
nothing went out (an EEPROM write refused under torque, a width that disagrees
with the register table, an encode the driver refused, too many IDs for one
frame) to `WireFailure::Unsendable`, and `PumpError::fault` then names the whole
of `PumpError::Bus` under torque `bus_failure`. So a defect of our own encoding
is published under the one word an operator greps for a wire fault, while
`PumpError::Map` — the same species of defect, caught one layer up — is
deliberately given no condition at all, on the argument that naming it
`bus_failure` would send somebody to the cabling over our arithmetic. The
response is not in question: nothing can be commanded, so torque comes off and
the machine parks either way. What is in question is the word, and the word is
the fault vocabulary's, so the answer belongs with the fault doctrine rather
than in this function: it decides whether an eighth condition gains a
qualification, a ninth condition is named, or a park-class ending is sanctioned
to carry no slug. None of the six is reachable from the move loop's own
transactions today — each would take a register-table or encoder defect to
produce — which is why it waits. Marked at the `Unsendable` arm of
`wire_failure` in `crates/reachy-bench/src/pump.rs`.

## `reachy-pod-motion-integration`

Host these crates from the pod payload rather than from the bench binary, and
decide how motion intents reach the loop.

Deferral context: the four libraries own no loop and no I/O by construction, so
whoever hosts them supplies the port, the clock and the loop. Today that is the
bench binary, which is a supervised operator tool: one command per process, a
loop that runs until the move finishes, and a fault that stops commanding and
exits. A payload wants none of those — it wants a long-lived loop taking intents
from elsewhere, and that raises questions this milestone deliberately does not
answer: what carries an intent, what happens to a fault when there is no operator
watching, and who owns the port when two things want it. None of it changes the
libraries; it is a second host beside this one. Marked at the driver in
`crates/reachy-bench/src/pump.rs`.
