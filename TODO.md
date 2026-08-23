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
Marked at `read_health` in `crates/reachy-bench/src/pump.rs` — out of the build
and deleted at the cutover under `bench-motion-delete`, so the marker moves to
whatever hosts the health sweep then.

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
`wire_failure` in `crates/reachy-bench/src/pump.rs` — out of the build and
deleted at the cutover under `bench-motion-delete`, so the marker moves to
whatever drives the bus then.

## `reachy-pod-motion-integration`

Host these crates from the cog compositions rather than from the bench binary,
and decide how motion intents reach the loop.

Deferral context: the four libraries own no loop and no I/O by construction, so
whoever hosts them supplies the port, the clock and the loop. The bench binary
was that host, and it is one no longer: its motion layer is out of the build, so
nothing in this tree drives a coordinated move on hardware today. The cog path
is the host being built, and the questions this entry waits on are the cutover
slice's: what carries an intent, what happens to a fault when there is no
operator watching, and who owns the port when two things want it. None of it
changes the libraries. Marked at the pod seam in
`crates/reachy-bench/src/pump.rs`, which is out of the build and goes at the
cutover under `bench-motion-delete`; the marker moves to the cog path where the
integration actually happens, and this entry is what carries it across.

## `bench-motion-delete`

Delete the bench's retired motion layer from disk.

Deferral context: `crates/reachy-bench/src/{commands.rs,pump.rs,trace.rs,
trace/metrics.rs,replay.rs}` are out of the build and no longer compile — and
will not against any version of the crates they name, since the vocabulary and
player types they import have since been deleted, so they are prose about the
machine rather than code anything revives. They are kept on disk as the record
of how this machine was actually driven — the sequencer call order, the fixed-rate loop, the settle policy, and
the hardware-trace replay that sizes the motion guards — which is the reference
the cog path is read against while it grows. They go when the cog path
demonstrably drives hardware, at the cutover; the trace recordings under
`crates/reachy-bench/fixtures/traces` outlive them and want re-pointing at
whatever replays them then. Precondition, and it binds before the cutover: while
`replay.rs` is out of the build nothing replays those recordings against the
`[motion]` guards they sized, so an edit to any of those bounds — step limits,
the tracking threshold, the gains, the antenna clocks — re-points the replay
suite at whatever hosts the loop first, and does not land on the strength of
`config.rs`'s range checks alone. A measurement nothing replays is folklore by
the next release. Marked in the header of each of those files, in
`crates/reachy-bench/src/lib.rs`, and beside the explicit `srcs` list in
`crates/reachy-bench/BUILD.bazel`.

## `session-hold-timeout-evidence`

Decide what the session does about the driver's `hold_timeout_torque_off` event.
It reads it and does nothing, and the fault vocabulary has no value for the
condition it reports.

Deferral context: the event says the goal stream went quiet for longer than the
driver waits and the machine was de-torqued because of it. That is evidence about
this host's own liveness rather than about the servos, and `FaultKind` names no
such condition -- so there is nothing to record it as and no response to
classify. The keep-alive rule is what makes the event unreachable in a healthy
run, so the decision belongs with that rule: either the session answers a hold
timeout as a condition (which wants a vocabulary value, and the numbering is the
log contract), or it is ruled a report about a bug and left to the driver's own
channel. Marked at `fault_of_event` in `cogs/session_ladder.rs`.

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
it is a context diff against a file every pin bump can move under it, and every
future consumer of rusty-cogs must carry a byte-identical copy of it and of the reasoning behind it. Not fixed here
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
(`counters!` reuses the state slot's setter identifier, so two totals declared
with each other's setter names compile; the slot crossing is
now pinned field for field by the round-trip case `counters!` emits, which leaves
unproven that a group setter reaches the signal named for it in the `.clk` and
that any value reaches the output log). Reading
one takes a Rust type bound to a group's generated schema, which
`rust_clk_module` is not established to emit; whether the emptiness is the
policy's reporting window, the run's length, or a drop limitation is the first
question. One statement is waiting on it by name: S6 is meant to show
`base_stretched` counting through the whole stack when a base plan stretched, and
a signal total is the only place that count appears -- so the stretch has cog-level
coverage and no end-to-end pin. Done when a scenario checker reads a
group's totals and asserts them against its own arithmetic. Marked at
`signal_groups` in `cogs/scenario/check.rs`, which every scenario of the motion
system calls.

## `resume-hands-back-the-path`

Build the running move once per control cycle. `resume` calls
`traj::read_seed`, which runs `Trajectory::new` -- two poses read out of
quaternion fields, a rotation inverse, a scaled axis and the finite and
duration checks -- purely to decide whether the bytes are a path, throws the
result away, and returns `Result<(), _>`; `motion_tick` then builds the same
object from the same bytes a few microseconds later. `resume` likewise
recomputes the targets and the seed pose the tick reads again.

Deferral context: the cost is small at 100 Hz and every future field-level
check added to `resume` doubles the same way, so it is worth removing. What
stops it being a local edit is that the tick may *write* the seed mid-cycle --
`take_command` starts a move, and `hold`/`abort` clear one -- so a path handed
in at the boundary is stale by the time the sampling runs. Removing the second
read therefore means the tick carrying an in-call second form of a value the
schema also holds, kept in step at every site that writes the seed, which is
the arrangement the schema-resident state rules out by design and a change to
two public entry points. Marked at `resume` in
`crates/reachy-motion/src/tick.rs`.

## `clip-schemas-config-home`

Move `ClipFrame`, `ClipConfig` and `ClipLibraryConfig` out of `cogs/config.clk`
into a clips-owned configuration module with its own protobuf generation, and
point `reachy-clips` at it instead of at `//cogs:config_clk_rs`.

Deferral context: the placement doctrine puts a schema in the package named for
the component that writes it, and `reachy-clips` is the only writer and the only
reader of these three. They sit in `cogs/config.clk` because that module was the
one crossing the Protobuf backend, and the whole module is one compilation unit,
so editing a simulator slew rate rebuilds `reachy_clips` and everything above
it — the inverse of the one-module-per-subject rule. What holds it is that the
frozen design assigns `cogs/config.clk` unchanged and rules that a declaration
moving to a different module than assigned is a design question, not an
implementer's call; the protobuf generation of a second config module also wants
proving before the move rather than during it. Marked at the clip schemas in
`cogs/config.clk`.

## `clips-authoring-split`

Split `reachy-clips` so the device build links only what plays a clip. The
playback half is `compose`, `config`, `player` and `speed`; the authoring half
is `format`, `library`, `vendor`, `files` and the importer binary, with `serde`,
`serde_json` and `anyhow` behind it.

Deferral context: playback now reads clips only out of the configuration
message, so nothing the running machine reaches touches the JSON document
reader, the loader, the vendor importer or the filesystem walk — yet every cog
build compiles them, and the crate's own header has to explain which of its
parts are at the crate's edge instead of the build saying so. The split is a new
crate with its own package and visibility boundary rather than a target-level
edit, which is a packaging decision the frozen design does not make, and the
cost grows each time the authoring side gains a format. Marked in the header of
`crates/reachy-clips/src/lib.rs`.

## `bazel-device-config-gate`

Cover the device configuration in a gate. The deployable is
`//crates/reachy-bench:reachy_bench` built with
`--platforms=//bazel/platform:reachy-device`, and nothing in `make check` or in
CI builds it: a `platform_transition_filegroup` (or a transitioned alias) beside
the platform definition would put it inside `bazel build //...`.

Deferral context: the shape is cheap; the cost is not. CI runs on an uncached
runner by design, so a second full aarch64 Rust build roughly doubles the
job — the decision is whether that goes into CI, into the local gate only, or
into a separate scheduled job, and it is a cost decision rather than a coding
one. What breaks silently until then: a dependency whose
`target_compatible_with` lacks aarch64, a `select()` a future dependency
introduces, or a drop upgrade that stops registering the aarch64 clang
toolchain — all of them discovered at `make bench-build` in front of a
powered-up unit. Marked at the `reachy-device` platform in
`bazel/platform/BUILD.bazel`.

## `ci-cache-refresh`

Decide whether CI's Bazel cache archive should refresh on every green run
instead of being written once per key, and if so what bounds its size.

Deferral context: the cache step keys on an exact dependency-graph fingerprint
with no `restore-keys` fallback, and `actions/cache` skips the save on an exact
hit. The archive under a key is therefore whatever the first cold run after a
dependency change produced, forever: third-party and toolchain actions are
served from it, but nothing this repo's own targets build — eight crates, the
clippy and rustfmt aspects over `//...`, the `.clk` compiles, the scenario
runners — is ever persisted, so every source-only push rebuilds and re-lints all
of it. The same freeze means a cache degraded by something outside the key, a
runner image bump changing local action environments, stays degraded until a
dependency bump happens to rotate the key, with nothing surfacing it. The
canonical fix is a rotating key plus `restore-keys` prefixes, which was
rejected because each save would then tar the prior key's stale entries
alongside the new ones and Bazel's own disk-cache GC runs at server idle, which
a CI job never reaches. That trade is what needs deciding rather than coding:
whether a prune before save (age- or size-capped) is a bound anyone wants to
own, against a per-run save of a multi-gigabyte archive and the 10 GB per-repo
budget it competes for. Nothing about it is a correctness question — both stores
are content-addressed, so any cache state produces the same build. Marked at
the cache step in `.github/workflows/ci.yml`.

## `sim-refused-readings-asserted`

Assert in the scenario checkers' standing set that the simulated driver left no
plant reading out of its register file, so a reading that stopped being a number
is loud at the scenario level rather than only in the cog's own cases.

Deferral context: a non-finite plant angle is counted and its cell keeps the last
finite value it held, so the published sample carries a plausible stale angle and
the count is the only evidence anything went wrong. That count is a state total
reported through the cog's signal group, and nothing in this repo can read the
value a signal carries — no Rust type binds to a generated report group
(`cogs-signal-report-contents`) and state slots do not reach the output log — so
a checker has no way to ask. The assertion is one line per checker once a signal's
contents are readable. Marked at `read_registers` in `cogs/sim_cogs.rs`.

## `session-servo-profile`

Give the servo-side velocity/acceleration profile the commissioning sweep writes
a measured value and a home in configuration, rather than a constant in the
session cog.

Deferral context: the pair in the tree (20 / 50 register units) is a modest
backstop chosen for a host that streams one step-bounded setpoint per period, and
nothing has run it on hardware. It is an order of magnitude below the figures the
bench carries in its own configuration, which are sized for a host that commands
whole moves outright, so the two cannot both be right for the same machine and
neither has a measurement behind it. What decides it is a hardware session — too
tight and the servos rate-limit a correctly shaped stream, which surfaces as
growing tracking error rather than as a refusal — and where the number then
lives is a policy question: the bench treats it as required configuration with no
default, and the online session has no configuration for it yet. Both halves of
that are outside what the deterministic runner can answer. Marked at `PROFILE` in
`cogs/session_bus.rs`.

## `aux-pending-carries-bustxn`

Let the session's pending-transaction record carry a transaction record whole,
rather than restating its fields, so the compiler owns the completeness of every
crossing a transaction makes.

Deferral context: the record a sequencer waits on is a schema of its own and the
session's pending record restates its five payload fields beside the correlation
number, the send instant and the re-issue count. A validated view is a reference
into the message it validated, so there is no value form of one to assign across
those crossings: each is field by field, and a field added to the transaction
schema would be dropped between the sequencer's record, the datagram and the
modelled bus in silence. Tests carry a fully distinct record through each
crossing and a tripwire fails when the record grows, which is what stands in for
the compiler today. The fix is a schema shape — the pending record holding the
transaction as a field, the way the driver's own slot state already does, which
makes every copy a whole-message one — and that is a change to a declaration this
arc's design froze. Marked at `Txn` in `cogs/session_bus.rs`.

## `session-mask-view`

Give the session a view of which joints the decision tick is still commanding,
so a wind-down can tell a head with nothing left to drive it from one that is
still being carried down.

Deferral context: the wind-down core asks its host two questions, and the session
can answer only one of them. Whether the machine reached the fold it reads off
the driver's pose stream; whether every joint that carries the head has been
taken out of service is a fact about the tick's mask, and nothing published
carries it. So the session answers `false` always, which is the conservative
reading — the stow keeps being commanded until the maneuver's own clock ends it,
where a wrong `true` would let go of a head that could still have been carried
down. What it costs is the record: a maneuver that ran out of joints is written
down as one that ran out of clock. The set could be assembled from the raises the
session already sees, and was not, because the tick also masks at the engage-time
health gate without raising, so an assembled set can disagree with the tick's own
— and disagreeing in the direction of `true` is the direction that drops a head.
The fix is the tick publishing what it is commanding, which is a channel and a
schema this arc's design does not name. The scenario suite feels it too: S8's
masked stow to park is asserted with the strict goal stream, because no joint
ever leaves service in that run -- so the "masked" half of the rung's name has no
end-to-end statement, and the day the mask reaches the goal stream that assertion
is where the run has to say which joint left. Marked at the evidence the maneuver
is stepped with in `cogs/session_stow.rs`, and at the goal stream S8 asserts in
`cogs/s8_checker.rs`.

## `commission-verdict-narration`

Give the timeline a row that says *why* the commissioning survey refused the
machine, so an operator reading the report stream learns which servo the sweep
did not find rather than only that the machine is parked.

Deferral context: a failed survey leaves the session parked without ever writing
torque, and what the record carries for it is a phase row from starting to parked
and nothing else. The verdict -- which rows answered, which register read what --
stands in the survey's own snapshot, which is a state slot and does not reach the
output log, so the report stream says "parked" and no more. The condition is not a
fault: nothing about a machine that is not the one this process was configured for
is broken, so it is not on the fault path, and the report vocabulary has no kind
for it. The fix is a report kind and a decision about which of the survey's
findings a reader is owed -- an addition to the log contract in
`motion/reports.clk` rather than a code change -- and it is the same decision
`engagement-declined-narration` is waiting on, so the two want taking together.
Marked where the survey's endings are read in `cogs/session_bus.rs`, and at the
narration S9 pins as it stands in `cogs/s9_checker.rs`.

## `engagement-declined-narration`

Give the timeline a row that says *why* an engagement was declined without ever
writing torque, so a sender that had its script accepted can tell a supply gate
that refused from a sweep that never completed.

Deferral context: an engagement that stops before its first enable write leaves
the machine limp and the session at rest, and what the record carries for it is a
phase row from engaging back to resting and nothing else. The condition itself is
not a fault — nothing about the machine is wrong when it declines to be armed on
the supply it has — so it is not on the fault path, and the report vocabulary has
no kind for it: the reasons live inside the sequencers as their own classified
failures. The fix is a report kind and a decision about which of those failures a
reader is owed, which is an addition to the log contract in
`motion/reports.clk` rather than a code change. Marked where the endings are read
in `cogs/session_bus.rs`.

## `tick-feedback-latch-composed`

Cover the decision tick's own feedback-lost latch end to end again: an outage
where the tick's tolerance for missed reads runs out before the driver declares
the bus failed, with the raise reaching the session, the session answering it,
and the goal stream ending because the tick gave up rather than because the
session let go.

Deferral context: S4 used to carry this, and the session taking hold of the
machine moved the scenario's subject — the driver's own bus-failure declaration
now reaches the session and parks the machine long before the tick runs out of
tolerance, which is the correct ordering and is what the scenario now asserts.
The composed statement that the two halves agree about a machine nobody can see
went with it, and it is not a statement a cog test can make: which of the two
notices first is arithmetic over two configured tolerances, which is what a
scenario is for. Arranging the other order needs either a way to suppress the
driver's declaration for a window, which is a new injection, or different
tolerances, which are motion-guard bounds this arc's design does not edit. Marked
where the deleted assertion stood in `cogs/s4_checker.rs`.
