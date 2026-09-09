# TODOs

Entries are slugs joined to `TODO(slug)` comments in the tree. See `CLAUDE.md`
for the convention — including that this file ships publicly.

## `example-placeholder` (DO NOT TRIAGE — this is a fake entry)

This is a placeholder entry. Leave it here so the file is never empty. It is not
a real TODO. You would reference it in code with a `TODO(example-placeholder)`
comment. That is the whole design: an entry here with a slug, joined to code
comments by that slug. Add real TODOs below this one, in this format.

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

Commission the body yaw and the antennas from measurement, as the legs now are.

Deferral context: the legs are done. A tour of the whole clip library with the
generator wide open read the class motor-bound at 326 velocity units with a
goal-step ramp median of 287, and a confirmation tour at `287 / 326` held the
six cranks 0.3319 rad from their own modelled trajectory at the worst, against a
0.4 rad bound and beside three earlier tours at the same pair at 0.3242-0.3288
rad. That pair is in `cogs/servo_profile.textproto` and in
`plant::SHIPPED_PROFILES`, its p99.9 floor is
`RECORDED_P999_LEGS_RESIDUAL_RAD`, and the tour's worst leg window is a kept
fixture.

The body yaw is measured and left where it is. The capability instrument reads
the class *gain-bound* at the shipped acceleration -- what holds it back is its
loop and not its motor -- and the reading it offers, `(20, 48)`, is inside the
instrument's own repeatability ratio of the shipped `(20, 50)`. So there is no
pair to commission: what headroom the class has is a gains question, which is
`TODO(body-yaw-gains)`. Its residual readings at the shipped pair span
0.3275-0.4035 rad over six tours, and one tour in three stands past the 0.4 rad
bound a candidate would be judged at; that is the class's own spread, recorded,
and not a capability reading.

The antennas are measured and *not* commissioned. Their capability is
`RECORDED_CAPABILITY_ANTENNAS`, `(522, 640)`, off a tour played at
`(32767, 1620)`. Three confirmation tours at `(522, 640)`, `(418, 512)` and
`(334, 410)` read the class motor-bound at the first two with a plateau above
the commissioned velocity -- the servo reaches its generator's speed -- and
found it standing 1.45, 1.51 and 1.94 periods of travel behind that generator
(p99.9 0.4458, 0.3707, 0.3813 rad; worsts 0.5078, 0.4238, 0.5989). A slower pair
shrinks that lag in radians while it stays the same in periods, down to a floor
near 0.38 rad -- where the third rung read the class gain-bound and the
instrument could no longer see the motor through the pair -- so stepping the
pair down is not an answer, and the tracking screen, sized at half again the
worst residual a healthy machine shows, has no room for the lag at the measured
pair. What stands between the two is the plant model: it carries a trapezoid and
no following lag (`plant.rs`' header lists what it leaves out), and a lag term,
or a dead time that grows with speed, is what would let the detector be armed at
the measured capability. Whether the screen's sizing rule should read the worst
sample or the worst sustained window is the same question's other half, since
the detector faults on the latter and `replay_test.rs` computes both.

Next step: that model, read offline against the kept tour logs at all three
antenna pairs and the recorded `20 / 50` tours -- no hardware run is owed before
it, because the analyzers walk kept logs -- and then one confirmation at the
pair the model is right at. The three-rung table is in
`docs/servo-tuning.md`.

Whoever commissions a pair also re-derives the prose figures evaluated at that
class's pair: the obstruction-cost paragraphs of `docs/fault-management.md` and
the tracking-detector sentence in `CLAUDE.md` -- the first raise's latency
(`crossing_cycles() + ticks`), the settled-move grace
(`ticks - pass_cycles(progress_min_rad)`), and the saturated-move grace, which
is the window less the periods a released joint's from-rest ramp takes to reach
`pace_min` of a generator at the cap: two at the legs' commissioned pair, four
at `20 / 50`, and its own count at any other. Every one of them is a figure for
content that runs the class at its profile velocity, and the documents say so.
The scenario suite is written over those expressions and moves by itself; the
documents do not, and no test reads a document.

The recorded residual figures are one configuration's reading per class and are
re-baked per class or not at all -- a fresh worst printed against a noise floor
measured under some other configuration compares two machines. The set is
`RECORDED_WORST_HEAD_RESIDUAL_RAD` and `RECORDED_WORST_ANTENNA_RESIDUAL_RAD` in
`crates/reachy-motion/src/tick.rs`, the three `RECORDED_P999_*_RESIDUAL_RAD`
arrays and the three `RECORDED_CAPABILITY_*` pairs in `cogs/pose_reading.rs`,
and the replay suite's own pinned worsts. The reading those comments defer is
still open: one recorded tour ran the body yaw to 0.4024 rad, so the threshold
stands at 1.491 times the largest sample on record rather than the 1.5 the
sizing rule asks for, and the tree ships that knowingly. The threshold is not
widened to close the gap; what closes it is a pair whose worst leaves the
margin, or a decision, recorded, that 1.491 is the margin this machine has.

Done = the body yaw and the antennas each either commissioned at a measured pair
or recorded as not commissionable under this detector, with the run that says so.
Marked at the profile fields in `cogs/config.clk`, at `SHIPPED_PROFILES` in
`crates/reachy-motion/src/plant.rs` and at the head residual pin in
`crates/reachy-motion/src/tick.rs`.

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

## `motion-proto-two-copies`

Two copies of the motion wire contract compile in this build: `crates/
motion-proto`, and the one `speech-surface` brings through the pinned host
closure. `//crates/reachy-host:host_closure_test` links the second.

Deferral context: `speech-surface` takes `motion-proto` from a *historical*
brenn-reachy commit, so the fetched copy is one publish behind this tree's and
re-pinning does not dissolve it. Harmless by construction: both seams the host
composes through carry an encoded body (`ScriptOut.body`,
`IntentSink::deliver(&str)`), never a `MotionScript` value, so the two types
never meet. What it costs is that a
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

Deferral context: `reachy-motord`, `reachy-host` (twice: the host itself and
`stt_compare`) and `reachy-ask` each spell the same shape — the `main` that dispatches to `parse`/`run` and prints `prog:
message`, the word loop, a `given` bool per flag, and a `usage()` string. The
refusal wording is operator-facing and already varies between the three for the
same mistake, and the per-flag bool scales with the flag count in every copy.
`reachy-host` now carries three flags — `--config` and `--speech-config`, both
path-bearing, and the bare boolean `--check`, which needed a `given`-style bool
of its own written a third time and a third arm that differs from the other two
only in taking no value. So the shape being copied is a loop over flags with an
arity each, not a loop around one, and the case for the `(flag, arity)` table
below is stronger than when this was written.

Not done in place for the same reason as `params-reader-shared`: one of the
three is the driver, which the cycle that made the third copy leaves alone. The
shape wanted is a small table of `(flag, arity)` with once-only enforcement and
one refusal vocabulary — a decision about a shared home and about whether an
existing dependency already carries a parser worth adopting.

Done = the four binaries parse their arguments through one helper and refuse
the same mistake with the same words. Marked at the harness's copy in
`crates/reachy-ask/src/main.rs`, at the host's in
`crates/reachy-host/src/main.rs`, and at the comparison tool's in
`crates/reachy-host/src/bin/stt_compare.rs`.

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

## `script-cause`

Give `Script` (`cogs/script.clk`) a field saying what caused it — a wake word,
or the sender's periodic refresh — set by the scripter that sends it, and let
the session refuse a refresh, but not a wake, that arrives at rest after a fault
ended the previous session.

Deferral context: the session cannot tell the two apart. A fault's ending now
refuses everything from the response through the wake its release confirms, so
nothing is applied to a machine still being stood down — but the sender refreshes
on a cadence of a few seconds, and the refresh after the release confirms is
indistinguishable from someone saying the wake word. A hand kept on the head is
therefore re-engaged roughly every other refresh, with no person acting between
cycles. Closing it is a contract change on the script and on the sender, plus
session state that remembers the last ending was a fault until a wake-caused
script clears it — a design cycle, not a patch.

Done = a machine stood down by a fault stays down until someone asks for it.
Marked at `phase_intake`'s `Resting` arm in `cogs/session_cog.rs`.

## `arm-record-retirement`

Decide whether the arming record survives: `motion/arm_record.clk`,
`crates/reachy-motion/src/record.rs`, and `arm.rs`'s `pin_goals`,
`pin_goals_from` and `PinOutcome`, which `lib.rs` re-exports. Either retire the
lot, or say in one place what it is being kept for.

Deferral context: the engagement is now one grouped driver cycle that takes no
pose from the session, which removed the last production caller of all of it.
`record.rs` is written and read only by its own tests, and the pose-pinning
helpers only by tests and by `record.rs` — so the reason each is kept is the
other one, which is a cycle between two dead things. It is left standing rather
than deleted because the frozen design for that cycle named these as staying,
and because the leg-window pull-in this record was written for may be the thing
that wants them back: whether a solved arming record is part of where this stack
is going is a design question, not a cleanup.

Done = the arming record has a live caller or does not exist. Marked at the
module header of `crates/reachy-motion/src/record.rs`.

## `hold-ordering-floor`

Decide what number a script held for a maneuver's ending is screened against.
Today it is the held script's own id alone (`hold`, `cogs/session_cog.rs`), so a
duplicate of the script the running engagement is already on is held and
narrated `script_held`, and then answered by the drain — `stale` if the maneuver
ends in `active`, an acceptance if it ends in `resting`. The same datagram
arriving in `active` is refused at once.

Deferral context: the frozen design for the pending slot states this rule — the
hold's floor is the pending id, and the drain applies the ordering rules of the
phase the maneuver ends in — so tightening it is a change to what the session
promises a sender, not a defect fix. The two candidate rules differ in what a
retransmit gets: a refusal alert at the edge one wake after a `script_held` row
that promised an answer, or an answer that depends on the phase. Deciding
between them is a decision about the sender's contract.

Done = one datagram gets one answer, whichever answer is chosen, and the rule is
written where the screen is. Marked at `hold` in `cogs/session_cog.rs`.

## `plant-chase-sequencer`

State the plant model's stepping sequence once. `PlantModel::step` is one
function every reader shares, but the loop around it — seed a row from a
reading, step against the setpoint of `RESPONSE_DEAD_SAMPLES` periods ago and
then push this period's, walk the periods no sample attended on the setpoint
already held, re-seed past `MAX_GAP_PERIODS` or when the driver holds nothing —
is written out five times: the decision tick (`advance_prediction`), the
simulated plant (`sim_cogs::advance`), the offline residual walk
(`pose_reading::residual_stream`), the scenario suite's travel walk
(`scenario::posture_walk`) and the replay suite (`replay_test::replay`).
Their agreement is what makes a run judged offline the run judged live, and
nothing but their comments asserts it.

Deferral context: the sequencer's state is slot-resident in two of the five —
the tick keeps it in `MotionSnap` and the simulated driver in `SimState`,
because it has to survive a restart — so one shared sequencer means each caller
marshalling nine predictions and a setpoint ring in and out of a schema every
period, on the control path. Whether that copy belongs on the tick's hot loop,
and whether the two schemas keep their present shape under it, is a design
question rather than a refactor. Until then each copy carries a comment saying
which walk it restates.

Done = one sequencer beside the model, and every reader of the plant calls it.
Marked at `RESPONSE_DEAD_SAMPLES` in `crates/reachy-motion/src/plant.rs`.

## `held-ring-pairing-pin`

Hold the setpoint ring and its count together by construction rather than by a
comment. `MotionSnap` carries the driver's last setpoints as `held` and how many
of them are real as `held_count`, and the two are one invariant: a reader that
cannot see the ring must not see its count either, or a count reads as a full
ring the record never wrote. The two field numbers were paired by hand, and the
rule that they move together lives in the `//` documentation element beside them
and nowhere else. Nothing structural or mechanical states it, so the next change
to the ring's layout can separate them again and only review would notice. The
argument is schema hygiene rather than an observed symptom: nothing in this tree
persists a `MotionSnap` across builds — the Mover's slot carries no
`TakeSnapshots` policy and a restarted process arms from a fresh slot — so the
separated pair has no reachable runtime symptom today, and `resume` refuses a
count past the ring's depth in any case.

Deferral context: the shape that removes the rule is one field — a nested record
holding the entries and the count, the way `tracking` already groups a
prediction with its run — which retires the count with the ring by construction;
it is also a second renumbering of two fields that have just been renumbered
once, and it changes what every Rust reader of the pair writes. The alternative
is a mechanical pin instead of a schema change, and the field numbers are not
visible from the generated Rust, so it would mean a check that reads schema
source — a kind of gate this tree does not have yet. Which of the two the schema
wants is a design call, and the invariant holds as written today.

Done = renumbering the ring without its count fails a build or a test rather
than passing review. Marked at `held_count` in `motion/tick_state.clk`.

## `refused-state-names-its-reason`

Let a refused control slot say which invariant it broke. `resume` answers a
`StateError` per way a slot describes no state a tick could be in — a mode
without its path, a clock that is not a length of time, a non-finite number in
a modelled trajectory, a held count past the ring's depth — and the Mover's
only reader is `if armed && resume(state).is_err()`, which drops the variant,
adds one to `refused_state` and re-arms from the next reading. Every kind reads
the same from outside the process: a counter that rose by one and a move
abandoned for a sample. The variants exist to tell a truncated or foreign slot
from a live defect, and that is the distinction an operator reading a run needs
most.

Deferral context: the Mover reports through numeric signals and a state field
per counter (`cogs/mover.clk`), so naming the reason is either a signal per
variant — a vocabulary that grows with the error enum and has to be versioned
with it — or a narration channel this cog does not have. Which of the two the
Mover should carry, and whether the same answer belongs on the session's
identical call site, is report surface and a design call rather than a
refactor. The refusal itself is safe as written: nothing is commanded and the
goal stream stopping is what takes the machine down.

Done = a run's record names which `StateError` refused a slot. Marked at the
`resume` call in `cogs/motion_cogs.rs`.

## `beam-gappy-segment-extent`

Let a closed segment's range be the span of the pod's index space it actually
covers. The run report attributes a turn whose span names no segment to the
closed segment whose `[base_sample, base_sample + samples)` range holds the
turn's carve. A segment with dropped samples in it spans more of the index space
than it holds samples of — the assembler's own successor part anchors on the
absolute index for exactly this reason — so such a segment's range stops short
of its last sample, and a carve near its tail is held by nothing and prints its
beam figure as missing.

Deferral context: the console does not say how many samples were dropped.
`segment_closed` carries `samples` (the assembled length) and `gap_count` (how
many gaps, not how long they were), so the true extent is not computable from
what the pipeline writes today; closing this means the pipeline stating the
dropped count, which is another repository's wire and a design cycle of its own.
The failure is a reading printed as missing rather than a wrong one, and every
segment in the runs fetched so far has `gap_count: 0`.

Done = a gappy segment's range covers its whole extent, and a carve in its tail
reads a figure. Marked at `Closed::samples` in `cogs/speech_run_report.rs`.

## `clip-one-pass-per-log`

Cut a run's turn clips in one pass per frame log. The run report resolves each
turn's carve on its own, and the resolver has no index to seek by: it decodes
every record from the head of the log and stops when it reaches the span's end.
Every turn of a session is carved from the same log, so turn *k* re-decodes what
turns 1..k-1 already decoded, and the work is quadratic in the turns of a
session. At the recorded store's configured cap — about 35 minutes, some 52,000
frames — an eight-turn session decodes roughly 200,000 frames instead of 52,000.

Deferral context: seconds, not minutes, at the session lengths run so far, and
the report is on the `make speech-run` critical path where that cost is
invisible. Closing it means splicing one pass into several output buffers, which
is an entry point beside `AudioSpan::resolve` in the pipeline crate — another
repository's public surface, and a pinned one — rather than anything this tool
can do over the API it has. The ceiling grows as the square of the turns per
session, which is the thing an acceptance run wants more of.

Done = a run's clips cost one decode of each log they come out of. Marked at
`cut` in `cogs/speech_run_report.rs`.

## `pod-ingest-test-util-in-host`

State, in the frame-log crate's own manifest, that its `test-util` feature is
enabled from outside it. The feature's comment there says it is never enabled in
a production build. This repository's dependency spec enables it — features are
a property of the package, not of the edge naming it — so every consumer in the
closure compiles the fixtures module, the voice host binary staged on the robot
included. The fixtures are inert today, so the cost is only that the rule the
upstream comment states is no longer the rule, and the next person to give a
fixture builder a dependency or a panic has nothing telling them otherwise.

Deferral context: the fix is one comment in the other repository, but that
repository's revision is the one this repository's pin is about to name, and
which revision is published and pinned is the operator's call rather than an
implementer's. It rides on that move.

Done = the frame-log crate's feature comment says who enables it and what
carries the module, and this repository's spec comment agrees. Marked at the
`pod-ingest` spec in `MODULE.bazel`.

## `stt-compare-shared-drain`

Call brenn-pod's published `speech_pipeline::transcribe_pcm` from `stt_compare`
instead of reading the transcriber's stream a second time here.

Deferral context: the voice host and this tool both hand a PCM buffer to a
`Transcriber` and read the stream for its settled transcript, and the tool's
whole claim is that it asks the recogniser the way production does. The reading
now exists once in brenn-pod, beside the trait whose stream contract it applies,
but the revision this repository pins predates it — which revision is published
and pinned is the operator's call rather than an implementer's, so it rides on
that move. Until then the copy here settles on the first final event exactly as
the shared one does; what remains is that the agreement is by hand.

Done = `stt_compare` has no stream loop of its own, and `futures` is named by
this repository only if something else still needs it. Marked at `transcribe` in
`crates/reachy-host/src/bin/stt_compare.rs`.

## `antenna-hold-gains`

Try an integral term on the antennas at the vendor's proportional term, against
the parking error the term-free loop leaves.

Deferral context: the hunt half of this entry is closed. The antennas were
walked over the step probes -- one commanded frame per pose, so the servo's own
generator makes and stops the whole move -- six rungs of six runs, every hold
judged by the stillness watch where the head stood still. 200 is the
proportional bound (300 hunts the rest hold, 400 the raised pose), the pose and
not the arrival selected the one hunt on record, and the fold that hunted at
every profile pair from the motor's ceiling to the shipped floor is quiet now
that the fold leans off the vertical. The user has ruled the remaining
occasional oscillation acceptable, so no further proportional or derivative rung
is owed.

What is left is the parking error. With no integral term the loop stops where
friction balances the proportional push: the right antenna parks 0.026 rad
(16-17 counts) short at the sides pose on the arrival from the fold,
analyzer-read at two profile pairs alike, on a hold whose excursion is inside a
count. A second reading of the same defect, 0.10-0.13 rad over 47 samples of a
kept tour, comes off a throwaway script and no tree instrument reproduces it: a
resting antenna under content whose goal moved by less than its breakaway error.
The instrument for the arrival reading is a probe run's `sides<-down` hold and
its signed mean error; the instrument for the content reading is a tour-side
parking reading -- over the tour's chasing samples with zero travel, per antenna,
the count, the signed error to the goal, how far the goal moved across each run
of such samples, and the head's travel over them -- which does not exist yet and
is this entry's own first piece of work, with its bring-up assertion being that
it reproduces the kept tour's 47 samples at 0.10-0.13 rad before any fresh tour
is read.

Done = both readings inside the stillness watch's two-count bound, or the cost
recorded as accepted. A rung is a number in `cogs/servo_gains.textproto` rather
than a code edit, and the library's own `DEFAULT_GAINS` is what that file is
pinned to, so a trial that lands is two statements moved together. Marked at
both: the antennas' triple in that file and `DEFAULT_GAINS.antennas` in
`crates/reachy-motion/src/arm.rs`.

## `body-yaw-gains`

Try a derivative term on the body yaw, and re-read the proportional climb under
it.

Deferral context: the yaw is the one class that is gain-bound rather than
motor-bound -- it reached 48 velocity units on a tour where the legs reached
326 and the antennas 640 -- so its ceiling is the loop rather than the motor,
and a stiffer loop is the lever. The P-only climb was walked and came back
non-monotonic: over the wake gesture's 17 s stow hold, 400 limit-cycles at
3.0 counts and 8-11 Hz apparent in three holds of three, 800 in one of three,
and the vendor's 200 sits at 2.0 counts of dither inside the two-count bound.
Damping is what a P-only ladder has none of, and it is what would let a higher
proportional term hold still, so the derivative term is the next rung rather
than a fourth proportional one. Nothing has complained about the yaw; this is
headroom left on the table, not a defect.

Done = either a derivative term is committed with the hold it was measured on,
or the record says a stiffer yaw loop is not available and the entry closes on
that. The instruments are `hold-probe <yaw id> --gains P,I,D` for the ladder
and a motion run that produces the long stow hold for the verdict. Marked at
both: the yaw's triple in `cogs/servo_gains.textproto` and `DEFAULT_GAINS.yaw`
in `crates/reachy-motion/src/arm.rs`.

## `antenna-raise-clock`

Give the antennas' raise its own clock, so the wake gesture can snap the
antennas into position without speeding the head's raise with it.

Deferral context: the wake raise is one duration. Head and antennas start
together on one command and the mover holds a single `up_duration_ns`, 0.8 s,
which the tick shapes into a min-jerk path for every joint -- so shortening it
speeds the head's raise on the six cranks too, which is a different move against
a different capability and not a taste knob. The servo profile is not the lever
either: nothing in the planner reads it, and what a faster pair changes is how
closely the servo follows the streamed path, not how fast the path is. At the
antennas' commissioned `20 / 50` the servo trails the 0.8 s raise by about two
seconds, so a shorter clock has nothing behind it until the antennas run a pair
they can follow -- which is `TODO(session-servo-profile)`'s antenna half. A snap
that is the antennas alone is a mover change with the detector's margin and the
head's timing in it, and it wants its own reading.

Done = a raise the user calls a snap, on record, or the ask withdrawn. Marked at
`up_duration_ns` in `cogs/mover_params.textproto`.

## `bench-probe-series-retention`

Decide what the device keeps of the hold probe's series, and make the fetch or
the bench enforce it: today every probe run writes a CSV into the account's
home and nothing ever removes one.

Deferral context: that home is a tmpfs, so the series are RAM the device gives
up nothing else for, and a tuning session is many runs by design — a minute-long
run is a few megabytes. The fetch no longer re-copies what it already has and
takes two connections whatever the count, so the cost that remains is device
memory alone. The two answers are not equivalent and neither is the tooling's to
pick: removing a series once it has been fetched makes the fetch the only copy,
and bounding what the device keeps drops the oldest evidence of a session while
it is still running. Both are decisions about how long a hardware reading
survives, which is the operator's call. The mark is at the listing in
`tools/deploy-bench.sh`.

Done = the device's retention is bounded by something, and the rule is written
down where an operator reads it.

## `capability-report-volume`

Decide how much of the capability instrument a routine run's report prints, and
print that.

Deferral context: the instrument reports every error band a chasing sample fell
into and every goal step the run wrote, per class, unconditionally — and both
analyzers call it on every run. At the shipped profile the antennas stand up to
about three radians behind, which is thirty bands, most of them under
`CAPABILITY_BIN_MIN_SAMPLES` and printed unread; a library tour's classes are
written dozens of goal steps. So the section a person skims after a bad run
carries the three or four figures they act on under a hundred lines they do
not, and it grows with the clip library. What it cannot become is a section
that drops readings: an unread band is a reading — it says the content never
held the class that far behind — and the step listing is what the dead time is
checked against, so which lines collapse, which stay, and whether the listing
belongs behind a condition is a decision about what a run's record has to
contain rather than a formatting tidy-up. Marked at `capabilities` in
`cogs/pose_reading.rs`.

Done = a routine run's capability section is bounded in length, and every
reading it stops printing in full is either still derivable from what it prints
or written down as deliberately dropped.

## `probe-clip-emitted-from-constants`

Emit the two antenna step probe clips from the poses they step to, instead of
holding those poses as hand-written frame literals.

Deferral context: `cogs/clips/probe/antenna-step-a.json` and `…-b.json` hold
about 650 copies each of two deltas — the fold and the sideways point, both as
differences from the rest pose — one per held frame, because the clip format
carries a value per frame and has no frame-repeat. The emit's own case checks
that the counts and the deltas match the constants, so a moved pose fails the
build; nothing writes the corrected document, and the last move of the fold
re-transcribed both files by hand. The fix is a decision about what
`make clip-config` owns: the emit writes the library textproto and the names
sidecar from the clip documents today, and having it author a clip document
instead makes a `cogs/clips/` asset generated output for the first time, with a
second choice — a frame-repeat in the format — that would change the format
every clip is validated against. Marked at
`each_antenna_step_probe_steps_to_three_held_poses` in
`cogs/gen_clip_config.rs`.

Done = moving either antenna constant regenerates the probe documents, or holds
the delta once, and no frame literal is transcribed by hand.
