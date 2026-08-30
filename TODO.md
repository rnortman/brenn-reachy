# TODOs

Entries are slugs joined to `TODO(slug)` comments in the tree. See `CLAUDE.md`
for the convention — including that this file ships publicly.

## `example-placeholder` (DO NOT TRIAGE — this is a fake entry)

This is a placeholder entry. Leave it here so the file is never empty. It is not
a real TODO. You would reference it in code with a `TODO(example-placeholder)`
comment. That is the whole design: an entry here with a slug, joined to code
comments by that slug. Add real TODOs below this one, in this format.

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

The host that had a health sweep is deleted. The marker sits at the rotation
that reads a servo's status registers, `crates/reachy-motord/src/aux.rs`'s
`health`, which is the path a driver's health reading now runs on: what it does
with a read nothing answered is publish no report and count the miss, and what a
run of them means is the undecided half.

The rotation is three reads per servo now, not two — the error byte, the supply
voltage and the present temperature — so a row that answers two of them and not
the third costs a report the same way a row that answers none does. That widens
the surface this decision is about without changing it, and the cycle budget the
three reads are charged against is `CycleBounds::of` beside the marker.

## `olog-schema-evolution`

Let the analyzer at HEAD read an `.olog` recorded under earlier schemas. Today a
schema append puts the whole fetched corpus behind a checkout: the reader binds a
channel by its recorded schema definition byte for byte, and every appended field
or enum value differs. Done is concrete: the report at HEAD decodes a fetched
records directory that was recorded before the most recent schema append.

Deferral context: the work is two halves, neither of them in this tree alone. The
`.olog` format already stores each channel's full serialized schema definition,
and the upstream half of the pinned Clockwork drop already ships a
decode-by-recorded-schema upgrader that its C++ and Python readers use — filling
an appended field from its declared initial value, keyed on the `version` and
`history` declarations the `.clk` grammar carries. So the first half is an ask
against the pinned drop: the Rust reader gaining that upgrader binding, which it
has none of. The second is this tree's: no schema here declares a history block
for an upgrader to key on, so the declarations have to be written and then kept
written with every append.

What absorbs the cost meanwhile is that the refusal is loud rather than wrong —
the reader turns an older recording away instead of misdecoding it — and that a
fetched records directory names the build that recorded it, so reading one is a
`git switch --detach` on the commit in its `provenance.txt`. And the refusal must
not be relaxed on its own whatever else happens here: a reader with no decode
engine that accepted a differing definition would read one schema's bytes as
another's, and an appended field changes the record size, so the payload does not
decode at any tolerance. The upgrader has to arrive before the check can soften.
Marked on `binding` in `cogs/log_read.rs`, the check that states the current
behaviour and is the refusal an upgrader-equipped reader would answer with a
decode instead.

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
produce — which is why it waits.

The host that classified them is deleted, and the vocabulary value they were
collapsed to survives as `WireFailure::Unsendable` in `motion/faults.clk`. The
marker now sits on that arm: `crates/reachy-motord/src/tick.rs`, at the read
whose transaction the port refused. Until the word is decided, the arm claims
the least it can — the rows are missing from the cycle's sample and the write
that frame would have carried is unconfirmed.

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
not the subset our targets reach. The copy is currently taken from the delimited
region of rusty-cogs' own `MODULE.bazel`, which took it from the drop, and both
hops are a manual diff. Two hand-copied
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

## `sim-aux-turned-away`

Give the simulated driver a second outcome slot, so a request it turns away
never displaces the answer of the one it served. The real driver publishes two
outcome datagrams on such a cycle — the served transaction's and the turned-away
request's `busy` — and the simulated one publishes whichever it wrote first.

Deferral context: the sim's outcome is a cog output port carrying one message per
execution, so the second answer needs either a second port or a second execution,
which is a change to the sim composition's shape rather than to the cog body
holding it. Until then a scenario that overlaps two out-of-band requests in one
cycle sees a driver that no longer exists: one answer, the other silent. Nothing
in the standing scenarios does that — the session is serial and holds one request
outstanding — so what the divergence costs today is a trap for the next scenario
that tries it. Marked at `Report` in `cogs/sim_cogs.rs`.

## `sim-aux-answer-record`

Hold the out-of-band answer record in one place. `Answer` — the struct, its
`op`/`id`/`reg` echo of the request, `bare`, `about`, `refused`, `busy`, `value`
and `write` — exists twice, in `cogs/sim_aux.rs` and in
`crates/reachy-motord/src/aux.rs`, with nothing linking the two, and `Request`
with it. So every status the vocabulary gains, and every field the outcome
echoes, is two edits in two crates that no compiler joins.

Deferral context: the copies have already drifted. The simulated host counts no
duplicate offers where the driver does, and it grants liveness before the slot is
offered the request, so a turned-away offer feeds the simulated dead-man where
the real one is not fed by it. Both crates already depend on `reachy-driver`, so
there is an obvious home beside `AuxSlot` — but putting a wire-answer record in
the crate that holds the driver's *decisions* and nothing of the wire is a
placement call about what that crate is, and the simulated host's independence
from the real one is the property that makes it a check on it rather than a
mirror. Which of those wins is a design question, and the liveness divergence
wants an answer of its own with it. Marked at `Answer` in `cogs/sim_aux.rs`.

## `session-servo-profile`

Give the servo-side velocity/acceleration profile the commissioning sweep writes
a measured value.

Deferral context: the configuration half is done — `profile_acceleration` and
`profile_velocity` are `SessionParams` fields, shipped as 20 / 50 register units
in `cogs/session_params.textproto`, and `check::commissioned_profile` pins the
file's values to the writes that reach all nine servos. What remains is the
measurement. The shipped pair is a modest backstop chosen for a host that streams
one step-bounded setpoint per period; the commissioning sweep has since written
it on a unit and the machine moved under it, but nobody has measured whether it
is the right pair. It is an
order of magnitude below the figures the bench ran (400 / 600, trial-validated,
including an 855°/s antenna sweep), which were sized for a host commanding whole
moves outright, so the two cannot both be right for the same machine. What
decides it is a hardware session — too tight and the servos rate-limit a
correctly shaped stream, which surfaces as growing tracking error rather than as
a refusal — which is outside what the deterministic runner can answer. Marked at
the profile fields in `cogs/config.clk`.

That symptom is now observed. The 2026-08-28 hardware runs of the wake gesture
report a worst head lag of 0.95–1.02 rad and a worst antenna lag of
2.47–2.52 rad, against fixture pins of 0.245 rad and 1.38 rad recorded under the
bench's 400 / 600 profile. Both of the gesture's commanded peaks (about
3.34 rad/s of leg crank, 7.57 rad/s of antenna) sit above the shipped 50's
1.20 rad/s cap and under the bench pair's 9.59 rad/s, and a cap-limited joint
chasing the min-jerk goal predicts roughly the lag that was measured. Consistent
with the rate limit, not measured as its cause: the session is what decides it,
and these are the before-numbers its after-numbers are read against — the lag
collapsing toward the pins under the trial-validated profile is the confirmation.
Nothing is failing on it: the tracking screen faults on lack of progress rather
than on distance, no fault fired in any of those runs, and the run report prints
the lag as a note it derives no verdict from.

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

## `repo-word-agreement`

Gate that `REPO` in `bazel/rust_clk.bzl` and the module name in `MODULE.bazel`
are the same word.

Deferral context: the word reaches the generator as `--repo` and heads every
crate name it derives, while a cross-repo importer spells that crate under this
repo's apparent name, which is its Bazel module name. The generator's
crate-name refusal cannot catch a disagreement — the macro and the generator
agree with each other and are both wrong together — so it surfaces as a missing
identifier at rustc in whatever tree imports us, far from the edit. Deferred
because the check is a new gate lane and this repo's gate has rules about
lanes: every tool it runs is pinned and nothing may skip when a tool is absent,
so where the lane lives (a `bazel test` target versus a `make check` step) and
how it reads a Starlark constant and a `module()` call without a parser is a
gate-design decision rather than a patch. Marked on `REPO` in
`bazel/rust_clk.bzl`.

## `motord-seam-trust-boundary`

Give the seam between the control process and the driver an access boundary, in
both directions.

Deferral context: the seam is UDP on `127.0.0.1`, six ports (marked at
`crates/reachy-motord/src/ports.rs` for the driver's two and at `PoseIn` in
`cogs/robot.clk` for the control box's four). Loopback restricts hosts and
nothing else, and it costs something different at each end.

The same two ports are marked a third time, at `ScriptsIn` in `cogs/robot.clk`:
the intent edge's incoming socket, whose sender is trusted for being on this
machine and built from this tree.

The intent edge's two ports, 7409 and 7410 (marked at
`crates/reachy-edge/src/ports.rs`), inherit the same boundary and the same
answer: a script sender that can reach 7409 can open an engagement, and a
sender on 7410 can narrate a session that is not happening.

Driver-side: any local process under any user can send a session command that
arms the machine and setpoints that move it, and those setpoints do not pass the
envelope check -- that check runs in the mover, upstream of the goal port. So
this is the one command path on the machine whose only guard against a violating
pose is that no untrusted code runs locally.

Control-side: a well-formed datagram on 7402-7405 from any local process is
indistinguishable from the driver's, so machine state can be spoofed (a stall
masked, a latch faked, the session's staleness watchdog fed while the real driver
is dead) and fault evidence fabricated or suppressed. The driver's hold-timeout
dead-man does not answer this: it fires on silence, and a sender's whole effect
is to prevent silence.

Ruled: the fix is a transport with a permission boundary — a unix-domain
datagram socket or equivalent, once the framework's socket layer can speak one.
The driver does not grow a second envelope check, and no token rides these
datagrams.

Why not the driver enforcing travel windows itself: the mover stays the sole
envelope authority. A second owner needs the kinematics the driver deliberately
lacks, and two copies of one rule diverge. And it would not close the hole
anyway — a local process that can reach these ports can still arm the machine
and command in-envelope motion no session asked for, so only an access boundary
answers the finding. A per-boot token in the schemas that cross the seam is out
for a second reason: it is a wire-format change on both ends, against the
framing decision that this seam carries the bare schema bytes.

Deferred, not declined: the transport is bigger than a patch and waits on the
socket layer.

## `shared-servo-fixture`

One scripted servo model behind the port seam, shared by every crate that tests
against one.

Deferral context: `crates/reachy-bench/src/testutil.rs`'s `FakeMachine`/`Spy` and
the `Machine`/`Shared` fixture in `crates/reachy-motord/src/tick.rs`'s test module
are two scripted nine-servo machines, and the bench one's own header says why it
exists — so the two copies cannot disagree about what a servo does with a write.
They already differ: the bench's answers a servo error byte, the driver's answers
a ping the bench's does not, and each crate's cases therefore cover a slightly
different machine. Promotion is not a change inside either module: it means a new
test-only library target that neither crate owns, deciding what of the bench's
`BenchConfig`/`Clock` coupling stays behind, and deciding whether the simulated
driver's plant — which is shipped code and not a fixture — folds into it or stays
separate. Marked at the driver's fixture.

## `driver-host-sample-glue`

Put the last of the two driver hosts' shared glue in one place: the
first-answer-wins rule for a cycle's outcome slot, and the gate-derived fields of
a pose sample.

Deferral context: the event a cycle raises, the ranking between two of them and
the blind-cycle counter now live in `reachy-driver`'s `report` module, and both
hosts read them from there. What is still written twice is smaller and harder to
move: `note_answer` in `crates/reachy-motord/src/tick.rs` and `Report::answer` in
`cogs/sim_cogs.rs` hold the same rule over two different answer types, one of
which is built over the bus layer, and the two `write_sample` bodies share only
their gate-derived half. Lifting either takes the transaction and pose
vocabularies into `reachy-driver` — and, for the answer, the bus dependency the
design deliberately keeps out of it — so what belongs there is a decision about
that crate's boundary rather than a move. Marked at `note_answer`.

## `online-host-logger`

Compose the logger box on a second `cpu_domain` — the dev host — and measure it,
so an online run's records land on real disk instead of on the unit's tmpfs.

Deferral context: the logger runs beside the control process on the unit today,
writing `.olog` files to `/run/brenn-app/logs/motion`, and the run's records are
pulled off with `rsync` after the run — scripted into `make motion-run` now,
which fetches before the operator can power the unit down. That works and it
keeps the deployment doctrine — nothing pushed to a unit touches its flash — but
it has two costs: a step between the run and the record, and a dependency on
tmpfs accepting the
`O_DIRECT | O_DSYNC` the framework's writer opens every file with. The writer
has no way to be told otherwise, direct I/O on tmpfs is a kernel-version
capability, and the unit's kernel is not the one this was verified on. The other
shape needs neither: the same box on the host domain, channels carried by the
framework's own TCP bridge once both domains sit in one `ethernet` block. What
it drags in is multi-node launch machinery — per-domain config generation, a
bridge process per domain, two process descriptions to start in the right order
— which is why it is not the shape the first hardware run uses. Marked at the
`RobotLogger` box in `cogs/robot.clk`.

## `host-run-in-ci`

Promote the host online run from a manual make target to a gated test.

Deferral context: `make motion-host-run` starts the whole online system on a
workstation — the real control process, the plant behind the real UDP seam, the
real logger — and judges the log it wrote with `first_motion_report`. It is
exactly the coverage the gate wants over the composition, the configs, the
launcher and the log format, and none of it exists in `make check` today.

What holds it back is measurement rather than design. One run is roughly half a
minute of wall clock (the budget is `run_seconds` in the harness), three
processes start in no order, and the spawn race is
absorbed by a delivery budget nobody has watched fail; the flake rate under a
loaded CI machine is unknown, and so is what a run costs there. It also binds six
fixed loopback ports and the empty-namespace shared-memory layout, so two runs
cannot share a machine — a gate has to say what happens when one is already
running. A handful of runs measures all of that; until then a green `make check`
should not depend on it. Marked at the header of `tools/host-motion-run.sh`.

## `mid-move-servo-condition`

A deterministic scenario in which a servo's error byte is read off the bus while
a maneuver is in flight.

Deferral context: S8 used to write the byte part way into the raise, and that
made the run's own arithmetic unstable — the driver's rotating read reaches the
faulted row somewhere inside a lap, so how far the head had risen when the fold
answering it opened depended on nothing the scenario stated. S8 now writes the
byte at a settled posture, and the suite's mid-maneuver arrival is the jam it
raises later, which is the decision tick's own evidence rather than a reading
taken off the bus. So the path where a *bus-read* condition schedules a response
over a maneuver already running has no scenario.

What it needs is a way to make the lap deterministic — placing the byte at a
cycle chosen from the rotation's phase, or an injection that lands on the faulted
row's own read — and an assertion written against a fold begun from somewhere
short of upright rather than from a fixed pose. Marked at the header of
`cogs/s8_scenario.rs`.

## `build-motion-test-flake`

Root-cause a single unexplained failure of `tools/build-motion.test.sh`.

Deferral context: on 2026-08-24 one full `make check` reported `87 passed, 1
failed` in this suite, and it has not reproduced since — four further full `make
check` runs and twelve direct runs of the suite, six of those concurrent with
each other. One failure in seventeen runs. The harness names the failing case
and the difference it saw on stderr, but the observer kept only the tally line,
so what survives of that run is the tally.
The suite is believed deterministic: fixed strings, forced mtimes, a stubbed
`bazel`, and an isolated temporary tree per case. So either that belief is wrong
somewhere — a leaked path, a clock read, an ordering between concurrent cases —
or the sighting was environmental.

What it needs is a second sighting, which is the reproduction nobody has. Until
one occurs there is nothing to bisect; when one occurs, investigate from it
rather than from scratch. The forensic half is now built rather than wished for:
`tools/test-lib.sh` keeps the staged tree of a run that failed and prints its
path, so a second sighting leaves the stubs, the mtimes and the payload layout
that produced it. This entry exists so that a second sighting starts from that
instead of from zero. Marked at the header of `tools/build-motion.test.sh`.

## `watchdog-holds-torque`

Decide what, if anything, answers an uncontrolled exit with the machine under
torque. The servos' own Bus Watchdog, armed at 200 ms by every session, does not:
a trip stops the servo and leaves torque held. Observed on hardware.

Deferral context: the driver's controlled wind-down de-torques on every stop it
can answer, and nothing answers the ones it cannot — SIGKILL, a crash, a yanked
cable — so the head stays where it was, holding its pose, until somebody powers
the unit down. The arming stays regardless: a stopped servo is better than one
chasing a stale goal. What to do about the torque is a fault-policy design cycle
extending `docs/fault-management.md`, not a change any one site can carry, and
the answer may be mechanical or procedural rather than code. Until that cycle
runs, the bench `watchdog` self-test's standing failure — it asserts a release,
which is what the policy requires, and fails on this hardware by design — is the
record, and it is not to be made green. Marked at the `bus_watchdog` comment in
`cogs/session_params.textproto`, where the armed value lives.

## `script-timebase`

Give a motion script an absolute start instant on a timebase both ends share,
so a step timeline can be written against the audio it accompanies.

Deferral context: step offsets are measured from the moment the receiving
process stamped the script's arrival (`crates/reachy-edge/src/intake.rs`).
Speech and motion therefore co-start only as closely as delivery allows —
whatever the hand-off costs is added to every offset in the timeline. That is
well inside the ±500 ms coarse coordination accepts, so a raise still reads as
an acknowledgement and a scheduled stow still lands over the tail of the audio.

It does not survive tighter coupling. Emote and gaze steps computed against the
audio timeline — a beat on a word, a tilt at a phrase — want the two timelines
to be one timeline, and offsets-from-receipt cannot express that: the receiver
has no way to know what instant the sender meant.

The schema already reserves the field. A `base` carrying an absolute start
instant makes every offset absolute, and what it needs underneath is a timebase
both ends agree on: a playout beacon pairing the audio sample clock to the
device's monotonic clock, a cog maintaining that mapping, and a script anchor
naming the utterance a timeline runs against. Blocked on that cycle rather than
on any one edit — it is a clock-distribution decision, not a field. Adding
`base` before there is a clock to interpret it against would put an absolute
time on the wire that each end reads differently, which is worse than the honest
offsets.

Done = scripts carry absolute step times on a timebase both ends agree on, the
executor runs against it, and offsets-from-receipt survive only as the fallback
when no base is present. Marked at the wire schema in
`crates/motion-proto/src/script.rs`.

## `host-status-egress`

A durable status surface for the voice host: what an operator reads to find out
what the machine is doing without following a console stream.

Deferral context: the retired motion daemon wrote a state file under `/run` and
`reachy-status.sh` read it. Nothing reproduces that. What the host has instead is
its JSONL narration — every row of the session's story, every body the edge
dropped, every alert the table raised — going to the launcher's per-app console
log, plus the alert plane for the few things worth interrupting somebody over.
That covers "what happened" and "wake me for this"; it does not cover "what is
the machine doing right now", which is the question a status command asks.

Deliberately narrowed rather than forgotten: a status surface is a decision about
where the answer lives (a file the host writes, a socket it answers on, a bus
query) and who reads it, and that decision belongs with the cycle that teaches
`reachy-status` the five-app payload rather than with the one that built the
edge. Nothing about the machine's safety rests on it — the session and the mover
decide everything, and the narration is a reader.

Done = an operator on the unit can ask what the head is doing and get an answer
that does not require reading a log. Marked at the host's console surface in
`crates/reachy-host/src/edge.rs`.

## `host-intent-producers`

The two tasks that author intent for the voice host's edge: the speech
pipeline's scripter, and the host's Brenn-bus subscription for remote senders.

Deferral context: the host holds the receiving half of the intent queue and the
gate behind it, and both are exercised. Nothing holds the sending half. The
scripter lives in brenn-pod's `speech-surface` and reaches the edge through a
sink seam on its `ScriptTask`; the bus deliveries reach it through an optional
motion-intent subscription on `BridgeDriver`. Both of those are brenn-pod
changes, and the wiring that injects them into this process arrives with the
pipeline composition — the half of `reachy-host` that is not the edge. Until
then the binary follows the session's story, narrates it, and asks for nothing.

The queue's own policy — bounded at eight, dropping rather than growing or
blocking — is sized against a presence refresh cadence no producer in this tree
runs yet, so the first real producer is also the first measurement of it.

The seams alone are not enough: both are reached through `Server::with_sinks`,
which is newer than the `BRENN_POD_REV` the closure is pinned at, so the wiring
cannot compile until that constant moves.

Done = a wake word on the unit reaches the motors through this queue, and a
script published on the bus's motion channel reaches the same gate, with
`MODULE.bazel`'s `BRENN_POD_REV` moved to a published brenn-pod revision
carrying `Server::with_sinks`. Marked where the sending half is dropped, in
`crates/reachy-host/src/main.rs`, and at the pin in `MODULE.bazel`.

## `motion-proto-two-copies`

Two copies of the motion wire contract compile in this build: `crates/
motion-proto`, and the one `speech-surface` brings through the pinned host
closure. `//crates/reachy-host:host_closure_test` links the second.

Deferral context: `speech-surface` takes `motion-proto` as a path dependency
inside brenn-pod at the currently pinned revision, and at brenn-pod's later
revisions it takes it from a *historical* brenn-reachy commit — so the fetched
copy is one publish behind this tree's either way, and re-pinning does not
dissolve it. Harmless today by construction: both seams the host will use carry
an encoded body (`ScriptOut.body`, `IntentSink::deliver(&str)`), never a
`MotionScript` value, so the two types never meet. What it costs is that a
decode-tolerance fix made here is not in the pipeline that produced the bytes,
and the two definitions can diverge with both repos' gates green.

The real dissolution is the scripter's migration into this repo, which retires
the back-pin entirely; that is a design cycle of its own. A byte-identity gate
test over the fetched sources would hold the line until then, and wants a
mechanism decision first — the fetched crate is a `crate_universe` repository
with no filegroup over its sources, so reaching them from a test is not the
one-line `data =` it looks like.

Done = `speech-surface` no longer depends on `motion-proto` from outside this
repo, and the build resolves the crate once. Marked at `BRENN_POD_REV` in
`MODULE.bazel`.

## `host-payload-membership`

A unit stages and starts `reachy_host`, but what it starts is not yet the voice
host: the pod audio device, the speech configuration and the wake/VAD model
files are not in the payload, and the process runs with the voice pipeline
unlinked.

Deferral context: the binary, its two configuration files and the two launcher
configs landed together, because the production config naming the host and the
harness twin that does not are one indivisible pair — a production config naming
`reachy_host` while `--run` starts `reachy_ask` against it puts both owners of
7409/7410 in one motion run, and that pairing had no reason to wait. What is
still missing all comes from brenn-pod or from the pipeline this binary does not
link yet: the `reachy-pod` audio device as a prebuilt artifact, `speech-surface`'s
TOML, and the openWakeWord and Silero models. `MODULE.bazel`'s `BRENN_POD_REV`
has to move to a published revision carrying `Server::with_sinks` before the host
composes the pipeline at all, so the runtime data it would read has nothing to
read it.

One thing is the payload's own and waits with them: the host will link ONNX
Runtime as a *shared* library — Microsoft publishes no static build, and the pyke
archive `ort` would otherwise fetch is in a container format Bazel cannot extract
— so `libonnxruntime.so.1` has to be staged beside the binary and found by the
loader. Nothing else in the payload has a shared-library dependency, so neither
deploy script has anywhere to put one yet;
`//bazel/third_party/onnxruntime:shared_object` is the label that names the file
(the device ISA sweep already reads it there). Staging it before anything loads
it would be a payload member nothing opens.

Done = the pod device binary, the speech configuration, the model files and
`libonnxruntime.so.1` are staged beside `reachy_host`, the loader finds the
shared object, and the production launcher config names the pod app as well as
the host. Marked at the filegroup in `bazel/platform/BUILD.bazel`, at the binary
in `crates/reachy-host/BUILD.bazel`, and at the ONNX Runtime archives in
`MODULE.bazel`.

## `params-reader-shared`

One textproto configuration reader, used by every binary that has one, instead
of a copy per binary.

Deferral context: `reachy-motord` and `reachy-host` each carry the same reader —
the embedded proto source, the descriptor compile and its cache, `load`/`set`/
`text`/`count`, the descriptor-walking transcription, and the generic half of
the refusal taxonomy (unreadable, not text, missing field, too long, schema).
Around 250 lines, differing only in the field arms and the domain checks. It has
already drifted once: motord attributes a text refusal to a line number and the
host does not, so one operator mistake reads two ways depending on which process
refused it.

Not done in place because the extraction moves `reachy-motord`'s reader, and the
driver is deliberately untouched by the cycle that made the second copy. What it
wants is a decision about where the shared piece lives — a crate of its own, or
a module in one that exists — taken with the third textproto-configured binary
in view rather than after it.

Done = one reader parameterised by proto source and message name, each binary
keeping only its field arms and its domain refusals, and the host getting line
attribution back with it. Marked at the host's copy in
`crates/reachy-host/src/params.rs`.

## `cli-argv-shared`

One argument grammar for the repo's binaries, instead of a hand-rolled `while
let Some(word)` loop per binary.

Deferral context: `reachy-motord`, `reachy-host` and `reachy-ask` each spell the
same shape — the `main` that dispatches to `parse`/`run` and prints `prog:
message`, the word loop, a `given` bool per flag, and a `usage()` string. The
refusal wording is operator-facing and already varies between the three for the
same mistake, and the per-flag bool scales with the flag count in every copy.

Not done in place for the same reason as `params-reader-shared`: one of the
three is the driver, which the cycle that made the third copy leaves alone. The
shape wanted is a small table of `(flag, arity)` with once-only enforcement and
one refusal vocabulary — a decision about a shared home and about whether an
existing dependency already carries a parser worth adopting.

Done = the three binaries parse their arguments through one helper and refuse
the same mistake with the same words. Marked at the harness's copy in
`crates/reachy-ask/src/main.rs`.

## `story-restart-discriminator`

A discriminator on `Timeline` that says which run of the control process a story
belongs to, so the edge's follower recognises a restart it cannot infer from the
row count.

Deferral context: the follower detects a restart by the story's total going
backwards, which is the only evidence a cumulative stream without an identity
carries. It is sound whenever the first datagram of the new story arrives while
its total is still below what was already narrated — the ordinary case, because
the stream publishes a datagram per appended row. It is not sound if enough of
those datagrams are missed that the new story has already grown past the old
count: the diff then reads the new story as a continuation, skips rows of it,
and neither narrates nor classifies them. Rows lost to a ring overrun have the
same shape, and both now raise the incomplete-narration Warning the alert table
carries, so the hole is loud rather than silent — but loud is not the same as
seen, and a fault row inside one still raises no Critical.

Not fixed in place because the fix is a schema field: a boot or epoch number the
session sets once and the follower compares, on a `.clk` message the cycle that
added the socket seam deliberately did not touch. Naming it, sizing it, and
deciding what sets it are a motion-schema decision rather than an edge one.

Done = the story follower tells one run of the control process from the next by
something the message says, not by arithmetic on its length. Marked at the
restart test in `crates/reachy-edge/src/story.rs`.
