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

## `fault-recovery`

Give the tick an explicit clear-fault command, so a machine that stopped
commanding can be told to resume without restarting the process.

Deferral context: a fault is absorbing — the tick emits nothing and ignores
every command thereafter — and the only way out today is to disarm, which drops
the head unless it is stowed, or to restart the process and re-drive the whole
arm sequence. Both are correct and neither is automatic, which is the property
worth keeping: the head is held up by the goals the servos are still holding, so
resuming is a decision a person makes with the machine in front of them. What is
missing is not the mechanism but the operator surface to issue such a command
deliberately and to show what is being cleared; the bench CLI runs one command
per process and has nowhere to put it. Marked at the absorbing arm of
`motion_tick` in `crates/reachy-motion/src/tick.rs`.

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

## `arrival-far-corridor`

Decide whether the far end of arming's arrival corridor stays at the arrival
tolerance, once the standing position error of a loaded servo on this unit has
been measured.

Deferral context: a joint the pins pulled somewhere else passes if it reads
anywhere between where it stood when torque came on and the goal it was sent to,
widened by the arrival tolerance at both ends. The near end is the direction
check — a joint moving away from its goal is fighting it — and costs nothing.
The far end is the goal itself, so a joint whose load pushes it *past* the goal
settles outside the corridor as soon as its standing error exceeds the
tolerance, and arming refuses a servo behaving exactly the way a proportional
loop with no integral term behaves. That refusal holds torque and stops the
session, so it is the safe direction to be wrong in, and no reading from this
unit says whether the case is reachable: the standing error is the figure a
supervised arm now records per servo and nobody has yet read one. Widening the
far end by a guess would put an unmeasured number in the one place the tolerance
was written to stay out of. Marked at the arrival check in
`crates/reachy-motion/src/arm.rs`.

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

## `held-goal-bound`

Decide what bounds the gap between a torque-holding servo's goal register and
its measured position on body yaw, where nothing bounds it today.

Deferral context: a servo found already holding torque is pinned at the goal it
is holding rather than at the position it has sagged to, so that re-arming does
not ratchet the target down by the sag every time. The gap between the two is
recorded per servo and judged on the legs only, by the pull-in gate, which is
measured against the position. Body yaw is pinned at that goal untouched, so
there the gap is bounded by nothing nearer than the envelope's yaw cap. What it
usually costs is
not a commanded slew — the servo already holds that goal, so writing it back
commands nothing new — but an armed record that claims the pose the goals
describe while the machine stands a sag away from it, and every trajectory then
starts from a pose the machine is not at. The exception is a servo whose torque
register says on while the servo is not holding anything: there the enable is
what commands the gap, and the goal-shadow gate that would have caught it is the
one thing a servo reporting torque is exempt from. The bound wants to be a
plausibility figure rather than a fence, and the sag it has to be plausible
against is unmeasured: it is the figure a supervised arm now records and nobody
has yet read one. Marked at the goal-shadow sweep in
`crates/reachy-motion/src/arm.rs`.

## `pin-settle-dwell`

Decide whether arming waits between writing the nine goals and reading back where
the joints ended up, and for how long.

Deferral context: the arrival check runs immediately — nine reads, a few
milliseconds — so a joint whose goal pulled it somewhere else is read while it is
still travelling. The check admits that: a pulled joint passes anywhere in the
corridor between where it started and where it was sent, so being mid-travel is
not a refusal, and a joint that was not pulled is compared against its own
reading a sweep earlier and races nothing. What the corridor cannot separate is a
joint that has stopped short of its goal from one still moving towards it, and
that is the case a dwell would close. Off the machine the two are
indistinguishable: a fixture can model an instant arrival or a slow one and
neither is evidence. What separates them is how far this unit's rest really sits
outside the travel windows and how long the profile-shaped pull takes, both of
which are readings from a supervised arm. A dwell also has a cost — it is time
with torque on and nothing verified — so the length wants to come from the
measurement rather
than from a guess. Marked at the arrival check in
`crates/reachy-motion/src/arm.rs`.

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

## `selftest-staleness`

Decide when a self-test record stops counting as evidence, and refuse to command
anything against one that has.

Deferral context: the record is what stands between an unverified machine and
every command that moves something, and today a record that passed every case
admits arming however old it is. Age is the obvious criterion and not obviously
the right one — a machine nobody has touched since the run is in the same state
it was, while one that has been unplugged, re-provisioned or taken apart is not,
and neither of those is a duration. What separates them is which facts the
record asserts that the arm sequence does not re-establish on its own, and that
list is short: the arm sequence re-reads presence, provisioning, supply, health
and the resting pose on every run. Settling it wants the first few bring-up runs
to show what actually goes stale in practice, and a criterion invented before
then would be a number nobody could defend. The record already carries the
timestamp any such rule needs. Marked at `SelftestRecord::admits_arm` in
`crates/reachy-bench/src/selftest.rs`.
