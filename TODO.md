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

## `override-regen`

Regenerate `bazel/clockwork_overrides.MODULE.bazel` from its upstream rather
than diffing it by hand, and gate the result.

Deferral context: Bazel honours a `bazel_dep` override only in the root module,
and every dependency the pinned Clockwork drop declares is versionless, so the
drop's whole override table has to be restated in this repo's root — all of it,
not the subset our targets reach. The copy is currently taken from rusty-cogs,
which took it from the drop, and both hops are a manual diff. Two hand-copied
tables in two repos is exactly the shape that drifts silently: the failure is
not a build error but a graph resolving at versions nobody chose. The fix is a
script that emits the segment from a drop's `MODULE.bazel` plus this repo's
named deviations, and a gate target that fails when the committed bytes differ
from what it emits. Marked in the header of that file.

## `rusty-cogs-pin-bump-merge-order`

Bump the rusty-cogs pin in `MODULE.bazel` past `e934ad2`, "fix: order the
offboard merge the way upstream's does".

Deferral context: the pin is `4e8f8e8`, the newest revision on rusty-cogs'
GitHub remote. Its local working tree already carries commits past that, the
first of which changes the order the offboard log reader merges channels in —
and that order is exactly what an ordering-sensitive assertion in a scenario
checker reads through (monotonic setpoint instants, strictly increasing
estimate times, event ordering). The reader at this pin is therefore known to
merge in an order that may not match upstream's tooling or the next pin, and a
checker written against it is written knowing that. Pinning past it is not
available until those commits are pushed. Marked on the `git_override` line in
`MODULE.bazel`.

## `foreign-generic-instantiation-crate`

Fix, upstream in rusty-cogs, the crate a generated dial names for an
instantiation of a generic declared in another repository's module — or record
here that our channels take their packet sizes off Clockwork's own menu forever.

Deferral context: the generator emits a generic instantiation into the crate of
the module whose `instantiate` statement created it, and attributes a reference
to one by whether the *referencing* module instantiated it too. A channel's
representation, though, must be instantiated in a module the cog module
*imports* — so for a `VarPacket<N>` at a size of our choosing the two rules
disagree: the type lands in our message crate and the dial spells it in
Clockwork's, and nothing links. Instantiating in both modules is refused
("layout already registered"). The working consequence is that this repo's wire
channels use sizes from `var_packet.clk`'s own `instantiate` menu (64, 128, 288,
…) and `//cogs/upstream` generates the Rust crate for that module. That costs
some slack in every packet carrier and nothing else, which is why it is a TODO
and not a blocker. Marked at the `instantiate` menu comment in
`cogs/upstream/BUILD.bazel`.

## `rusty-cogs-root-only-include`

Drop the `include()` from rusty-cogs' own `MODULE.bazel` upstream, inlining the
`bazel_dep`s its `.bzl` and `BUILD` files need, and delete
`bazel/rusty-cogs-patches/` here.

Deferral context: `include()` is legal only in a root module, so rusty-cogs as a
dependency fails during main-repository-mapping computation — before any target
is analyzed — and this repo carries a patch that rewrites that line into ten
versionless `bazel_dep`s. The patch is exactly the fix, in the wrong repository:
it is a context diff against a file the pin already needs to move (see
`rusty-cogs-pin-bump-merge-order`), and every future consumer of rusty-cogs must
carry a byte-identical copy of it and of the reasoning behind it. Not fixed here
because the change belongs to the sibling repo, which this slice does not touch.
Done when rusty-cogs resolves as a dependency unpatched and the patch package is
deleted. Marked on the `patches =` line in `MODULE.bazel`.

## `rusty-cogs-macro-parameters`

Give rusty-cogs' `rust_clk_module` the parameters that force this repo to copy
it — the repository name, the runtime and generator labels, and per-call `repo`,
`root` and `crate_name` overrides — then `load()` theirs and delete the copies.

Deferral context: `bazel/rust_clk.bzl` (336 lines) and `bazel/BUILD.bazel`'s
`framework_clk_imports` filegroup are verbatim copies of rusty-cogs artifacts at
the pinned revision, copied because `REPO` is a file-level constant heading every
crate name the generator derives and because the macro names the filegroup label
in the module being compiled. Re-syncing means re-applying four named changes
across 336 lines at every pin bump, with no gate: a copy that drifts still
builds, and the divergence surfaces as a crate name no importer spells.
`cogs/upstream/BUILD.bazel` is the same cost paid a second way — the generator
invocation and its `rust_library` written longhand because the macro derives what
that module needs spelled out — and every further upstream module our cogs carry
messages from adds another hand-written copy that must track the generator's flag
set. Not fixed here because the fix is in the sibling repo, which this slice does
not touch; until it lands, the fallback is `override-regen`'s shape — a target
that diffs the copies against the pinned upstream files modulo the named changes.
Marked in the header of `bazel/rust_clk.bzl`, at the filegroup in
`bazel/BUILD.bazel`, and at the longhand invocation in `cogs/upstream/BUILD.bazel`.

## `rusty-cogs-signal-trampoline-linkage`

Give the signal trampolines rusty-cogs' C++ shim emits internal linkage
upstream, then drop the patch this repo carries at
`bazel/rusty-cogs-patches/shim-trampolines-internal-linkage.patch`.

Deferral context: the shim writes one `extern "C"` trampoline per signal method,
named `signal_<cog index>_<method>` -- unique within its own file only -- inside
an anonymous namespace that its own comment says is there "so that nothing links
against a trampoline". C language linkage defeats that: the symbols are external
and unmangled, so two cog modules' shims linked into one process define the same
names and the link fails. That blocks any process hosting cogs from more than
one module, which is what `cogs/sim.clk`'s box is. The patch replaces `extern
"C"` with `static`, which is what the anonymous namespace was reaching for, and
changes nothing else. Not fixed here because the change belongs to the sibling
repo, which this slice does not touch. Done when two cog modules link into one
executable against an unpatched rusty-cogs and the patch file is deleted. Marked
on the `patches =` list in `MODULE.bazel`.

## `clockwork-single-instance-signal`

Drop the `multi_instance: true` marker from every signal in `cogs/motion.clk`
and `cogs/sim.clk`, once a single-instance signal compiles in a system whose
generating module declares a box and whose system target registers over it.

Deferral context: the marker is a fact about the build rather than about the
machine. The compiler builds a casing per box a generating module declares *and*
another for the system target over it, and each casing registers an instance of
every signal its cogs carry; a signal not marked `multi_instance` refuses to
compile with more than one ("Signal ... is not marked as multi_instance and
already has an instance"). Two module layouts were tried to avoid it — one box
per generating module, and the process in a module of its own — and the system
target's own instance defeats both. Upstream's heartbeat example carries the
same marker for the same reason. The cost is that every signal any future cog
declares must carry it or the build breaks, and that an offboard consumer is
told to expect several reporters of a quantity that has exactly one — so
aggregating `goals_published` across "instances" is a question with a wrong
answer available. Not fixed here because the fix belongs to the sibling
framework, which this slice does not touch. Done when the marker is dropped from
both files and `//cogs/...` still builds. Marked at each cog's `signals` block in
`cogs/motion.clk` and `cogs/sim.clk`.

## `clockwork-build-updater-strips-annotations`

Let a package hold a rendered rule with attributes and comments of its own, then
mark `//cogs:motioncpu.textproto` (and its counterpart in `cogs/proof/`)
`tags = ["manual"]` so wildcard builds stop building a launcher config nothing
consumes.

Deferral context: a `clk()` over a system module brings a gate test that
re-renders the package's BUILD file and diffs it, and the renderer emits the
`system_sim_clk` and `motioncpu.textproto` rules with no comments and no extra
attributes. Adding either to the launcher config's rule fails
`//cogs:system_sim_clk.build_test` with a diff that removes it again, so the
target cannot be annotated where it is. What it costs today: every clean
`bazel build //cogs/...` merges a config no target reads and builds
`@clockwork//clockwork/pinion:tcp_bridge_main` to do it, and the same target
will be rendered into every future system package. Not fixed here because the
fix belongs to the sibling framework's renderer, which this slice does not
touch. Done when the launcher config's rule carries `tags = ["manual"]` and the
build test passes. Marked in the header of `cogs/BUILD.bazel`, beside the
paragraph describing the rendered rules.

## `cogs-signal-report-contents`

Assert a scenario's signal totals against the run they describe: decode one
report group out of the output log and check `goals_published`,
`samples_seen` and `goals_executed` against the scenario's own cycle counts.

Deferral context: nothing in this repo reads a signal's value. The cogs declare
three groups, the box gives each a `ReportGroupPolicy`, and the groups do reach
the output log as channels -- carrying, at this drop, no messages at all over a
five-second S1 run. So neither half of the surface is covered: not that a total
reaches the group, and not that each total reaches the signal named for it
(`counters!` and `sim_cogs::signal` reuse the state slot's setter identifier, so
two totals declared with each other's setter names compile and round-trip
through the state slot, which is what every existing assertion reads). Reading
one takes a Rust type bound to a group's generated schema, which
`rust_clk_module` is not established to emit; whether the emptiness is the
policy's reporting window, the run's length, or a drop limitation is the first
question. Done when a scenario checker reads a
group's totals and asserts them against its own arithmetic. Marked at
`signal_groups` in `cogs/scenario/check.rs`, which every scenario of the motion
system calls.

## `cogs-session-channel`

Give the session cog its own channel to the driver. It will need to send the
driver traffic of its own — a torque-off control datagram, and the aux
transactions an arming or disarming sequence is made of — and `DriverCmd`, the
goal stream, is not where that can go.

Deferral context: every Clockwork channel at this drop has exactly one
publisher. An input fed by a channel two cogs publish to needs `no_dial`, and no
system holding a `no_dial` input loads at all, so the shape the parent design
sketched — the decision tick and the session cog both publishing goals and
controls on one channel — is not implementable. Nothing is lost yet: this slice
has no session cog, and the decision tick is `DriverCmd`'s only publisher. The
decision is which shape the session slice takes — a second channel into the
driver, or a multi-publisher channel if the framework grows one — and it is that
slice's to make, with the aux path's requirements in hand. Marked at the
`DriverCmd` declaration in `cogs/motion.clk`.
