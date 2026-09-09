# Servo tuning — capability, per-class profiles, and the antenna hold

What this document is: the procedure for tuning the servos' profile and gains
against what the motors actually do, and the record of every run that has been
read. Safety: `docs/fault-management.md`. The bench and deploy mechanics:
`docs/bench-runbook.md`.

## Why a profile is tuned at all

The commissioning sweep writes one `(profile_acceleration, profile_velocity)`
pair per class into the servos, and the motion tick models each joint's motion
with a trapezoid built from that same pair. The tracking detector judges a joint
against that model. So the pair is two things at once: the servo's own
trajectory generator, and the host's prediction of where the joint should be.

The consequence that shapes every step below: **the model is a valid prediction
only while the generator is the binding constraint.** A pair above what the
motor achieves under load makes the trapezoid describe something the motor is
not doing; the residual then measures motor lag rather than obstruction, and an
armed detector faults a healthy machine. Tuning is therefore not "find the
biggest pair that works" — it is: measure what the motors achieve under the
library's load, set each class *under* that with margin so the generator binds
again, and confirm the residuals stay inside the screen.

Three classes, because their motors and their recorded Velocity Limits differ:
the six platform legs, the body yaw, the two antennas.

## The knobs, and how a run carries them

Three files are configuration a run can vary:

| file | what it sets |
|---|---|
| `cogs/servo_profile.textproto` | six scalars: acceleration and velocity per class |
| `cogs/servo_gains.textproto` | nine scalars: P, I, D per class |
| `cogs/mover_params.textproto` | among them `tracking_armed` |

**The experiment overlay.** `REACHY_EXPERIMENT_DIR` in `.local/reachy.conf`
names a directory. At `--push`, every file under it is overlaid onto the staged
payload at its payload-relative path — `<dir>/cogs/servo_profile.textproto`
replaces the payload's. Only those three paths are accepted; anything else
refuses the push. Each overlaid file is named on the console with its sha256.

**Every push**, overlay or not, copies those three files into the run's log root
as `config/<payload-relative path>`, and `provenance.txt` gains a
`config_sha256=` line per file and an `overlay=` line. `--fetch` brings the log
root back whole, so a fetched run carries the configuration that produced it,
and both analyzers read the run's own `config/` rather than the tree's. A log
with no `config/` is refused: read it with its own build.

Two consequences worth knowing before an experiment:

- `tracking_armed: false` is a **`fail`** in every report, always, with no flag
  to excuse it. A disarmed run never reads green; it is read for its notes.
- A report whose configuration differs from the analyzing tree's says so at the
  top. An overlay forgotten in `.local/reachy.conf` is loud on the next
  ordinary run.

## The instruments

- **Capability and health**, in `first_motion_report` and `library_tour_report`.
  Per class, in this order: how many samples the joint spent chasing a setpoint
  more than `CHASE_GAP_RAD` away and the per-period travel over them
  (p10/p50/p90/max, in rad and in Profile Velocity units) against the recorded
  Velocity Limit; the class's own `RECORDED_CAPABILITY_*` pair for comparison;
  **the travel cut by the tracking error it was made at**, one 0.1 rad band per
  line with its sample count and travel percentiles, every band printed whether
  read or unread; the **regime** those bands read as, with the figures it turns
  on; the increase — chasing periods that travelled at least a count further
  than the period before, p50 and p90 in Profile Acceleration units; and **the
  goal-step listing**, every step the class was written with the joint's travel
  over the periods after it and that step's ramp, then the class's ramp median.
  Per servo: first, last and peak temperature with when, voltage range, and the
  worst error byte the run latched. Notes, not verdicts.

  The three regimes are the whole reading, and each names a different binding
  constraint:

  - *motor-bound* — the travel stopped rising with the error. Two or more
    adjacent bands whose medians sit inside `CAPABILITY_PLATEAU_RATIO` are the
    plateau, and the speed in it is the motor's rather than the content's.
  - *gain-bound* — the fastest band read is more than that ratio above the band
    below it, so the speed was still rising with the error when the content ran
    out. The motor was never reached; what bounded the joint was its own
    position loop.
  - *content-bound* — fewer than two bands left to read. The library never held
    this class far enough behind for two speeds to compare, and the run says
    nothing about the motor.

  The regime is read off the bands below any *fall*. A servo at its ceiling
  holds a speed as its error grows; it does not lose speed with more error, so a
  readable band whose median sits under the band below it by more than
  `CAPABILITY_PLATEAU_RATIO` is never a ceiling reading. It is the first periods
  after a large goal step — the joint a whole move behind, still in its dead time
  or its ramp — or a stall. From the top down, each such band is set aside until
  a band reads one speed with the one under it or is the faster of the two; a
  band whose median read zero falls away by the same test. Each band set aside
  gets its own line with its figures, the ratio it fell by (or that it read
  zero), and that it is read past; the regime line then says how many bands it
  was read under.

  A class that reads *content-bound with bands read past* is no measurement of
  that class: the run held it far behind and those far bands are the ones set
  aside, so the line says the reading needs two speeds, how many bands were read
  past, and that the run offers no candidate.

  A band under `CAPABILITY_BIN_MIN_SAMPLES` prints unread and is read past, and
  a class written fewer than `CAPABILITY_STEP_MIN_SAMPLES` goal steps prints its
  listing with no ramp median at all.
- **The residual's sign**, in both analyzers' residual section: beside each
  class's worst and p99.9, the worst the class stood *behind* its own modelled
  trajectory and the worst it stood *ahead* of it. A joint behind its model and
  a joint ahead of it call for opposite answers to the same figure — stepping a
  pair down widens the second — so step 3's branches read the sign before the
  magnitude.
- **The stillness watch's period line**, in the `stillness` section: mean and
  spread of the sample count between reversals, and the apparent frequency at
  the window's own sample rate. A regular limit cycle reads as a small spread; a
  bit of encoder dither reads as a spread comparable to its mean. The apparent
  frequency is an alias — at a series sampled at `r` Hz an apparent `f` is any
  of `|r·k ± f|`.
- **The settle line**, under each printed hold in both analyzers' `stillness`
  section: what the joint did over the 4 s allowance the judged window drops —
  peak-to-peak counts, reversals per second, the apparent period — and the
  words *not judged*. The allowance covers the travel to a far goal and the
  coming to rest after it, so a joint that arrives hard swings and rings down
  inside it; the verdict stays the judged tail's, and the settle line is how a
  ring-down is told from a joint that never swung. A hold whose allowance the
  recording holds nothing over prints no line.
- **The `stillness` section of the tour report**, over the same measurement the
  motion run prints, with one rule the motion run does not need: an antenna
  hold is **judged only where the head stood still across it** — no head row
  commanded somewhere new between the antenna's setpoint change and the hold's
  last judged reading. A hold under head motion is printed with its figures and
  `under head motion, not judged`, because what it read is the rod following
  the platform it is mounted on and the two-count bound is written for the
  antenna's own loop; those holds run 7–24 counts at either gain on record. A
  tour that held the antennas only under head motion says it measured nothing
  about them and fails nothing — a content tour holds them almost only while
  the head moves, which is the content's shape. What this judges under the
  content tour is the one head-still rest hold per antenna, and under a probe
  run every one of the probe's arrivals. A **probe** run holding none is the
  one case where that absence *fails*: a probe's holds are head-still by
  construction, so a report of none is a stimulus that did not arrive, and a
  green run there would make the rung's *quiet* verdict true over no holds at
  all. Which of the two the run is read off the table it was asked for — a
  table of `probe/` instruments alone is a probe run's.
- **`make bench-run ARGS="hold-probe <id>"`** — holds one servo where it stands,
  under torque, moving nothing, and samples Present Position in a tight loop at
  around 1 kHz: one phase with the goal rewritten every 20 ms as the driver does
  and one phase reading only. Prints per phase the peak-to-peak counts, reversal
  rate, the reversal intervals and the dominant period from the autocorrelation
  of the differenced series, and writes both series as
  `hold-probe-<stamp>-<id>.csv` beside the self-test record, fetched by `make
  bench-fetch` — which brings the series back whether or not a self-test has
  written a state file beside them, and refuses rather than overwrite a local
  copy of a name holding different readings. `--gains P,I,D` swaps the position gains for the run and
  restores them on every exit path. It refuses a servo holding torque and a
  servo whose Bus Watchdog is armed — the second phase is seconds of the silence
  that register stops a servo for, and a servo halted by it would read as the
  quiet phase this probe exists to find. `off` releases torque and writes
  nothing else, so it does not clear an armed watchdog: `reboot <id>`, or a
  `watchdog <id>` run, which disarms on every exit path.
- **The antenna step probes**, `probe/antenna-step-a` and
  `probe/antenna-step-b` in the clip library. Two documents that drive the
  antennas and nothing else, so the head holds the raised base bit for bit
  while they play. Each visits three poses off that base — antennas up (the
  rest lean itself), out to the sides (horizontal, `ANTENNA_OUTBOARD`), and
  down (stow, `±3.32`, leaning 10.2° inboard of straight down) — and each pose
  is authored as the difference of the two constants it puts the antennas at, so
  a retuned pose moves the probe with it. Each pose is reached in **one frame**
  and then held 6.5 s,
  which is the stillness watch's shortest judgeable hold and half a second.
  A one-frame jump in a tracked channel is a single far goal written in one
  period, so from there the servo's own profile generator is the whole
  trajectory: the arrival is as hard as the commissioned pair makes it, which
  is the stimulus no gesture this stack plans for itself produces. Probe a goes
  up → sides → down → up and probe b up → down → sides → up, so between them
  every arrival is judged once. Played at 1.0x and at nothing else — a
  fractional speed interpolates between frames and the step becomes a ramp.
  Both documents also author **no entry blend**. A blend is a weight ramp over
  the whole delta a frame carries, and although a probe opens on the base, the
  ramp is then spent on the frame that carries the first pose: at the format's
  200 ms default the first arrival of each probe would reach the servo as ten
  periods of a tenth of the pose, which is a shaped move and not a step.

  **The down pose is where the hunt is judged.** The wake gesture holds the
  same pose at the end of its fold — head stowed, antennas down, torque on —
  but that hold lasts about 2.8 s from the stow's last setpoint to the release,
  short of the 4 s allowance plus 2 s minimum the watch wants, so it is counted
  and never judged; the record of that pose is the re-read of the kept runs
  below. Each probe holds the antennas at stow for 6.5 s with the head standing
  at the raised base, arrived at over the servo's own profile, and the tour
  report's head-still rule judges exactly that hold — three times per rung per
  arrival direction. The head's posture is not part of the measurement: what is
  asked is what the antennas do when they are pointed down and stopped hard.

  Both antennas sweep through their **outboard** arc, each through its own
  side's horizontal. That is the arc planned moves are routed away from,
  because it sweeps the widest envelope around the machine exactly where
  objects sit beside it; it is taken here so the pair cannot interfere with
  each other mid-sweep, and it is why a probe run needs clear space beside the
  head as well as above it.

  **How one is played.** `make motion-probe MOTION=probe/antenna-step-a`:
  `library-run`'s chain over one motion — build, push, one script, fetch,
  `library_tour_report` — with the records landing under `probe-log-<stamp>`
  and the table the run was asked for written into the run directory beside its
  `config/` copy. About a minute of motion.

  **A probe is not tour content.** `make library-run` plays the library minus
  every motion whose name starts `probe/`, and the analyzer judges the tour
  against that same selection. A probe is an instrument: it writes a step goal
  no clip in the library holds, and the tour is the content the recorded
  fixtures the detector's screen is sized against come off. So a probe is
  played one at a time, deliberately, and never as one leg of a tour. The rule
  is the sender's: it makes the selection for the plan it runs and prints the
  same one for the verdict, so the two cannot disagree.

## The procedure

Each step names its pass rule before it runs. Every accepted figure is committed
as configuration or as a pinned constant, with its run directory recorded below.
**A figure is baked only where the step's decision tree, written before the run,
says that reading bakes it.** A reading the tree did not name stops the step, and
nothing from the run is baked until the reading has been explained and the ruling
recorded here beside the figure; the baked figures are reviewed with this record
beside them before they ship.

### 0. Offline baseline

**Done** — the figures are under "The record" below, and both are baked in
`cogs/pose_reading.rs`.

No hardware. `library_tour_report` over the kept tours and `first_motion_report`
over the kept motion runs, with the tree's three configuration files staged as
each log's `config/` — that is what those runs ran on. Two figures came out that
nothing had measured before:

**A kept tour is judged against the name table of the commit that recorded it**
— `git show <commit>:cogs/clip_library.names.json` — for the same reason its
`config/` is staged from that commit. A
motion id is the table's own numbering over the library's sources, so adding an
instrument to the library renumbers everything that sorts after it: the two
antenna step probes moved `bench/sway`, `bench/tip` and `bench/tour` on by two.
Judged against a later table, a kept tour reads with every window after the
insertion attributed to the wrong motion and fails on motions it was never
asked for, and nothing in the output says the table is the wrong vintage. Runs
fetched since `--probe` landed carry no such question: the harness writes the
table the run was asked for into the run directory as `asked.names.json`, and
that is the one to judge them against.

- **Temperature**: per servo, the peak a healthy tour reaches and how far in.
  `TEMPERATURE_STOP_C` is set between that peak and the servo's own 70 °C
  shutdown, with the margin stated against the recorded figure, and is a `fail`
  rule in `health_summary`.
- **Residual distribution**: per class, worst and p99.9 under the shipped pair.
  The worst was already pinned; the p99.9 is `RECORDED_P999_*_RESIDUAL_RAD`,
  three figures per class rather than one, and every residual line prints the
  range.

### 1. The antenna gain sweep, over a hard stop

**Done, and closed.** The antenna triple is the vendor's `200 / 0 / 0` and the
gain question is not open: no triple this machine has been swept at is quiet at
every pose, and what answered the oscillation was leaning the fold, not a gain.
The sweep, its rungs and its verdicts are under "Antenna gain sweep — the hard
stop" below. What the step is for, and how it would be walked again, is here.

**The stimulus is a step probe, not a gesture.** Nothing this stack plans for
itself stops an antenna hard: the tick shapes every move as a min-jerk path and
emits one sample per period, so the profile registers are the backstop under a
goal step the host got wrong rather than the thing that paces the motion, and
the wake raise runs the mover's own clock at any pair. A clip's frame, by
contrast, is content: a one-frame jump in the antenna channel is a single far
goal written in one period, and from there the servo's own profile generator is
the whole of the trajectory. `probe/antenna-step-a` and `probe/antenna-step-b`
are that stimulus — antennas-only documents stepping between up, sides and
stow, each pose held 6.5 s with the head standing bit-for-bit at the raised
base. Between them they cover every arrival of the sequence up, sides, down,
sides, up, and both direct up↔down arrivals.

Every rung is one overlay directory holding `servo_gains.textproto` with the
rung's triple, and `servo_profile.textproto` with the rung's antenna pair
wherever it is not the tree's, and **three `make motion-probe` runs of each
probe, six in all**. No overlay carries `mover_params.textproto`: an overlay
file replaces the tree's whole, so a stub carrying only `tracking_armed: true`
would drop the rest of the file and panic the process, and the tree's value is
already armed and falls through.

**What is read, per rung**: the run's own `library_tour_report` stillness
section under the probe-run standard, which judges an antenna hold only where no
head row was commanded somewhere new across it — so all four holds of a probe
run are judged — plus the health section and the residual section, which is what
the armed detector judged live. The record carries per hold the excursion in
counts, the settle line, the signed mean error and the arrival pose, and per run
the worst signed antenna residual and health.

The verdicts, named before the ladder runs:

- *Quiet*: every judged antenna hold's excursion inside the two-count stillness
  bound, three runs of three, both probes.
- *Parked*: quiet, **and** `|mean error|` inside that same two-count bound at
  every judged hold. The bound is the encoder's own flicker, the one figure this
  stack already holds a still joint to; nothing looser is invented for it.
- *Hunts*: any judged hold over the bound in any run. One hold is enough — the
  same rule the body yaw's `800` rung was judged by. What tells a hunt from
  dither is the period line: a limit cycle's reversal intervals have a spread
  small against their own mean, dither's does not.
- *Faulted*: an obstruction report or a `TickFaults` row in any run. The antenna
  did not follow its own generator at that triple under the plant model. The
  rung fails, its residual table goes into the record, and the ladder does not
  step further in the direction that produced it.
- *Health* out of band at any rung: stop and record, as step 2's reading 1.

Three ladders, in order, D held at 0 throughout: A0 walks the arrival, halving
the antennas' acceleration from the recorded pair while the velocity stays; A1
walks the proportional term upward from the first quiet arrival, doubling, with
one bisection under the rung that first hunts; A2 walks the integral term upward
at the chosen P, stopping at the first *parked* rung. Each ladder's exits are the
verdicts above, and the record carries every rung's table whether or not the
ladder reached an outcome.

**What the sweep found, and what it did not.** The pose selects the hunt, and
the arrival does not: the fold hunted at 7–8 counts in the same 10–13 Hz band at
the motor's own pair, at three halvings of it and at the shipped `(20, 50)`,
while the settle travel moved by seconds. So A0's ladder had no lever, A1 had no
quiet rung to climb from, and A2's premise — a parking deadband at the rest hold
for an integral term to close — is falsified at every `I 0` rung on record. The
proportional bound is `200`: the rest pose hunts at `300` and `400` and the up
pose hunts at `400`, with the cycle's amplitude growing with P at both.

**The answer was the pose.** Straight down is the mirror of straight up —
gravity loads no side of the gearbox play and the loop hunts across it — and the
rest pose has leaned ten degrees against that mechanism since it was tuned.
`STOW_ANTENNAS` now leans by the same magnitude, 10.2°, and inboard, which also
tucks the pair; at the leaned fold the same rung is quiet in six of six.

The bound is never widened, and a hunt this procedure cannot quieten is not
made to disappear by a looser one. What is left, if a pose ever hunts that a
lean cannot answer: a derivative term above the vendor's P, current-based
position mode, feedforward gains, or resting the antennas de-torqued — and that
choice is the user's. The one gain rung still deferred is the integral term at
`P 200`, against the parking error alone, carried by
`TODO(antenna-hold-gains)`.

### 2. Motor capability

One overlay: `servo_profile.textproto` with each class at the top of Profile
Acceleration's own range, 32767, and its velocity at the class's recorded
Velocity Limit (the head's 445, the antennas' 1620), `tracking_armed: false`,
and the tree's gains. Then `make library-run`, **attended, with the space
clear**.

Read, in this order:

1. **Health.** Any non-voltage error bit, or any servo at `TEMPERATURE_STOP_C`,
   is the ceiling: that class cannot play the library at its own speed, and its
   candidate is bounded by the pace at which the temperature was still flat —
   the p50 over the first third of the tour rather than the whole. A servo that
   faulted stops the reading: the machine is at the Minimum Risk Condition, and
   that class takes the p10 figure with the fault recorded against it. Read the
   per-servo *rise* too — first to peak — against the healthy 1 to 3 C a tour
   produces at the shipped pair. A rise well past that with the peak still under
   the stop figure is written into the record against the pair; it is not a
   verdict, because no aggressive tour has been measured and there is no figure
   to set one against.
2. **Capability, per class.** The regime the error bands read as says which rule
   applies, and the report names it. Every figure is rounded down.
   - *Motor-bound*: velocity is the **smallest median in the plateau** in
     Profile Velocity units — the slowest speed the class held while its speed
     had stopped rising with its error, which is a floor a healthy motor clears
     in every direction and under every load. Acceleration is the **median of
     the class's goal-step ramps** in Profile Acceleration units: the second
     post-write period's travel less the first's, over every step the class was
     written. It is read off the steps and not off the increase because the
     increase over chasing samples sits at about a third of the ramp the same
     joints show on a goal step, and a pair written from it would ramp the legs
     in nine periods where the motor does it in three — a model that is slow is
     followed easily, and step 3 would pass it.
   - *Gain-bound*: the motor was never the constraint, so there is no plateau to
     read. Velocity is the median of the `[2 · CHASE_GAP_RAD, 3 ·
     CHASE_GAP_RAD)` band — the speed the loop made at twice the chasing gap,
     the largest error the content produced often enough to read — and
     acceleration is the p50 increase.
   - *Content-bound*, and only where **no band was read past**: the library
     never held the class far enough behind to measure it. Velocity is 1.25 ×
     the maximum observed per-period travel, capped at the recorded Velocity
     Limit; acceleration 1.25 × the maximum increase. The rule is an
     extrapolation above everything observed, and its premise is that the class
     was never held far behind.
   - *Content-bound with one or more bands read past*: that premise is false —
     the class **was** held far behind, and those far bands are the ones the
     report set aside as ramp or stall — so the extrapolation does not apply and
     the class has **no candidate** from that tour. The reading stops the step
     and goes to a person; an extrapolation here would hand step 3 a pair above
     the motor, which is the plant model the armed detector judges against.

   A class with fewer than `CAPABILITY_STEP_MIN_SAMPLES` goal steps has no ramp
   median, so a motor-bound class in that position has no acceleration figure
   the steps measured and takes the increase instead, with that stated against
   the pair.
3. **Repeatability.** Fly the same tour a second time under the same overlay and
   read both with the same instrument. A plateau velocity, a gain-bound band
   velocity, or a ramp median that differs between the two tours by more than
   `CAPABILITY_PLATEAU_RATIO` is a reading this step does not name: it stops the
   step before step 3.
4. Write the candidate pairs into a second overlay carrying the pairs alone; the
   tree's `mover_params.textproto` falls through, armed. Then go to step 2a if
   any class's gains are open, else to step 3.

This run's residual figures are meaningless by construction — the model was the
wrong one on purpose — and its verdict is a `fail` on the disarmed detector. Its
notes are what is read. Its capability figures are baked as
`RECORDED_CAPABILITY_*` constants, each read against the figure this record
already holds for the class — a re-read outside `CAPABILITY_PLATEAU_RATIO` of it
is a reading the step does not name and stops it.

### 2a. A gain ladder for a class the tour read gain-bound

A class that reads gain-bound has an open question the tour cannot answer: its
candidate is the speed its *loop* made, so a quieter, stiffer loop would move
the candidate. The ladder is only worth walking where the gains are open — where
nothing has measured them against a hold — and it is walked before step 3, so
that step 3 confirms one configuration rather than two.

Per rung, one overlay carrying `servo_gains.textproto` only, the class's P
doubled and `I`/`D` at zero, and `make motion-run` three times at the shipped
profile with the detector armed. **The ladder's rows are the antenna rows and
the judged class's own row**: `cogs/stillness_report.rs` judges only the
antennas and folds every head row to one unjudged worst hold, so a head class
under this ladder is read from that folded line, hold by hold.

**Pass**: every watched hold inside the two-count bound, three runs of three,
and the widest hold's period line carrying the dither signature — reversal
intervals whose spread is comparable to their own mean — rather than a limit
cycle's near-constant interval. **Fail**: one hold over the bound with the
regular-interval signature, in any run. One hold is enough and no bound is
widened.

- *A rung passes* → it is the candidate, and its triple is committed with the
  ladder's figures in the comment.
- *Every rung fails* → the class stays at the vendor's value, nothing is
  committed, the ladder's table goes into the record, and the next lever — a
  derivative term at a P above the vendor's — is deferred as a TODO with its own
  instrument: `hold-probe <id> --gains P,I,D` for the probe half, and a motion
  run that produces the long hold every time for the confirmation half.
- *The ladder is non-monotonic* — a higher rung quieter than a lower one — is a
  reading, not a reason to prefer the higher rung. A rung on which the machine
  showed a limit cycle hunts, whatever the rungs around it did.

### 3. Confirmation

`make library-run` then `make motion-run` under the candidate overlay, armed,
with the classes that are not candidates falling through to the tree — the
configuration that ships if it passes, and nothing else. A candidate carried
beside a pair that is not shipping makes the bake mixed-provenance, which is a
comparison of two machines.

**Pass**: no tick fault, no obstruction report, every window moving, health
flat, and for each **candidate** class a worst residual at most **two thirds of
the 0.6 rad tracking screen, 0.4 rad** — the sizing rule's own margin, since the
screen is 1.5 × the worst sample over the kept fixtures and a bake that failed
that assertion is a bake that cannot ship. The p99.9 is printed against the
baseline's and read, not judged: a p99.9 that grew more than the worst did is a
motor lagging everywhere rather than at a few extremes, and is written into the
record beside the pair. The residual is read with its sign, because the two
signs take opposite branches.

A class the tour plays at the tree's pair is a **control**, not a candidate: its
worst is read against its own record and a reading outside that record by more
than one period of travel at its pair is a stop, while a reading inside it is
recorded as one more sample and bakes nothing. Its capability line is recorded
and not judged, because the walk cannot read a motor through a generator written
below it — a class at `(20, 50)` reads a plateau at the generator's own cap, and
comparing that with a `RECORDED_CAPABILITY_*` figure read at `(32767, 1620)`
compares two different measurements. A class with no record at the shipping
configuration is read against the 0.4 rad margin instead, which the shipping
configuration has to clear whether or not its pair moved.

Stillness is printed and recorded per hold — both the motion run's section and
the tour's head-still-judged one — and does not gate. The analyzer's non-zero
exit on a stillness finding is not a confirmation failure: the antennas' gain
question is closed on the record below, and the lever a stillness fail used to
reach for (halving the antennas' acceleration) is measured inert over thirty
runs.

- **Pass**: commit each candidate's pair to `cogs/servo_profile.textproto` and
  `SHIPPED_PROFILES`; re-bake the recorded worst residuals by cutting this
  tour's worst windows as new fixtures, each replayed under the profile it was
  recorded at and each with a row saying so; re-derive the prose figures that
  quote the old pair's timings, each with the regime it is evaluated under;
  record the margin against the shipped pair's. The tour's capability re-read of
  a candidate class must land inside `CAPABILITY_PLATEAU_RATIO` of the pair it
  ran at; a plateau above the candidate by more than the ratio is recorded and
  **one** further iteration at the re-read pair is allowed, so that the loop
  closes. A second excess is recorded as headroom left, not chased.
- **Fail on residual, the joint behind its trajectory**: stop, and take the
  residual-against-pair table to a person. There is no ladder down. A class that
  reaches its generator's speed — motor-bound with a plateau at or above the
  commissioned velocity — and still stands a fixed number of *periods of travel*
  behind it at two different pairs is a **following lag** in the position loop,
  not a trapezoid faster than its motor. A slower pair shrinks such a lag in
  radians while leaving it where it was in periods, down to a floor no pair
  moves, and past that floor the readings get worse and the walk stops reading
  the motor at all; the answer is a plant model that carries the lag, which is a
  design question and not a pair. Read the class's p99.9 in periods of travel at
  the pair it ran at, and its capability regime, before reading its worst in
  radians.
- **Fail on residual, the joint ahead of its trajectory**: escalate with the
  residual table and the goal-step listing. Stepping the pair down would widen
  the reading, not close it.
- **Fail on health**: as step 2's reading 1.
- **A class read content-bound with any band read past** on the tour: the tour
  is not a measurement of that class and nothing is baked for it.

The gates that stand either way: no threshold widened, no bound widened, no
figure baked that the step's decision tree did not name.

## The record

Each entry names the log or record directory, the configuration it ran under,
and the figures read out of it. Every step of the procedure has been run: the
offline baseline, the antenna hold probes, the body yaw's gain ladder, the two
capability tours, the antenna gain sweep over the step probes, and the
confirmation. The gain sweep and the confirmation each closed on a reading the
step's own decision tree did not name, and each entry below says what the
reading was and what was decided instead.

A reading is marked **[A]** where an analyzer in this tree produced it — a
report section, or the bench's own probe output — so that re-running that
analyzer over the kept log reproduces it. It is marked **[S]** where it came from
a throwaway script over a `//cogs:trace_export` of the same log; those scripts
are not preserved, so an **[S]** figure is reproducible only by re-deriving it,
and no code in the tree cites one.

### Offline baseline

Both analyzers built at `676a7d7`. Each log's inner run directory had
`cogs/servo_profile.textproto`, `cogs/servo_gains.textproto` and
`cogs/mover_params.textproto` from the tree at `676a7d7` staged as its
`config/`.

**Tours** (`library_tour_report`), all under `.local/motion-logs`:

| tour | written at | detector at that commit |
|---|---|---|
| `tour-log-20260906T190015Z` | `dd5bb33` | none — no tracking module in the tree |
| `tour-log-20260906T210529Z` | `cbcaea2` | none — no tracking module in the tree |
| `tour-log-20260907T174348Z` | `4725511` | present and armed by construction |

The staged profile and gains are what each of those commits held — `20 / 50`
in one pair, gains legs 800/100/300, yaw 200/0/0, antennas 500/0/100 — and the
clip library the three tours played is identical, so these are three runs of one
configuration. `cogs/mover_params.textproto` is staged for a different reason:
the analyzer refuses a log whose `config/` lacks it, and `tracking_armed`
postdates all three runs, so the staged file attests nothing about their
detector state; the table's third column is git's answer.

`tour-log-20260906T190015Z` is read here although the design's step 0 named only
the two later tours: it is a third run of the same profile, gains and library,
and the p99.9 is read for growth, which needs a noise floor. The recorded-fixture
tours the design also named are CSV trace fixtures of the replay suite, which the
tour analyzer does not read; their `20 / 50` rows remain what that suite asserts
over.

**Motion logs** (`first_motion_report`): `motion-log-20260907T184533Z`,
`20260907T173458Z` and `20260906T210658Z`. Reading these with this build prints
two schema notices — `SessionCmdChan` and `DriverAuxOut` were re-shaped this
cycle — which make any kept motion log's verdict non-green under it. That is the
expected cost of reading an old record with a new analyzer, not a defect; the
pose, health, event and fault channels bind in every case, which is everything
the baseline reads, and the three tours print no notice at all.

**Temperature, per servo, over the three tours** (first → last, peak and how far
into the tour):

| servo | 190015Z | 210529Z | 174348Z |
|---:|---|---|---|
| 10 | 30 → 31, 31 C (296 s) | 30 → 31, 31 C (221 s) | 29 → 30, 30 C (198 s) |
| 11 | 32 → 33, 34 C (386 s) | 31 → 33, 34 C (428 s) | 31 → 33, 33 C (338 s) |
| 12 | 30 → 32, 32 C (220 s) | 30 → 32, 32 C (191 s) | 30 → 31, 32 C (222 s) |
| 13 | 29 → 31, 31 C (389 s) | 29 → 31, 31 C (357 s) | 29 → 30, 30 C (110 s) |
| 14 | 29 → 31, 31 C (233 s) | 29 → 31, 31 C (307 s) | 29 → 30, 31 C (334 s) |
| 15 | 31 → 34, 34 C (372 s) | 31 → 33, 34 C (428 s) | 31 → 33, 33 C (322 s) |
| 16 | 33 → 35, 36 C (379 s) | 33 → 35, 35 C (184 s) | 32 → 35, 35 C (392 s) |
| 17 | 33 → 34, 34 C (103 s) | 33 → 34, 34 C (103 s) | 33 → 33, 34 C (178 s) |
| 18 | 35 → 36, **37 C** (274 s) | 35 → 36, **37 C** (197 s) | 35 → 36, 36 C (91 s) |

- **The healthy peak is 37 C**, servo 18, twice in three tours. Voltage 7.2–7.6 V
  throughout and the only error bit anywhere is input-voltage, expected on this
  machine. Ambient was not recorded, which argues for a wide margin.
- **The healthy rise is 1 to 3 C per servo over a seven-minute tour** of the
  whole library. A 35-second motion run (184533Z) moves no servo more than 1 C
  and peaks at 35 C.
- `TEMPERATURE_STOP_C = 50` (`cogs/pose_reading.rs`) — 13 C over the peak, more
  than four tours' worth of heating in one run, and 20 C under the Temperature
  Limit register's 70 C, so the analyzer's ceiling is read before the servo's own
  protection acts. It fails any run, and it is the ceiling step 2's health
  reading uses.

**Residual distribution, per class, under the shipped pair** (worst / p99.9):

| class | 190015Z | 210529Z | 174348Z | recorded p99.9 range |
|---|---|---|---|---|
| body yaw | **0.4024** / 0.2754 | 0.3884 / 0.3264 | 0.3275 / 0.2488 | **0.2488–0.3264** |
| legs | 0.2932 / 0.2290 | 0.2978 / 0.2190 | 0.2796 / 0.2131 | superseded, below |
| antennas | 0.3795 / 0.2999 | 0.3782 / 0.2759 | 0.3855 / 0.3177 | **0.2759–0.3177** |

All figures in rad, all read at the measured dead time of two samples. The
head pin, 0.3884 rad, is tour 210529Z's body-yaw figure, which the analyzer
reproduces; the antenna pin is no longer this table's 0.3782 but the
confirmation tour's 0.3913 rad, read at the `200 / 0 / 0` gains the class ships
(the `### Confirmation` record below). The p99.9 is
recorded as a range and not one number because step 3 reads a candidate's
p99.9 for growth, and the run-to-run spread between identically configured
tours — 0.078 on the body yaw, 0.016 on the legs, 0.042 on the antennas — is
the noise floor of that reading. A candidate inside the range has not grown;
one above its top has, by at least the amount above.

The legs' recorded range is no longer these tours'. The class is commissioned at
`287 / 326`, and a floor measured under a pair the class no longer runs compares
two machines, so `RECORDED_P999_LEGS_RESIDUAL_RAD` is the legs' p99.9 over the
three tours at the commissioned pair — 0.1651 / 0.1674 / 0.1650 rad, a range of
0.1650–0.1674, off `tour-log-20260909T015956Z`, `…T020934Z` and `…T021726Z`. The
three cells above stand as what the legs did under `20 / 50`. The antennas' range
is these tours', at the `500 / 0 / 100` gains they ran, which the machine has
since moved off; the body yaw's is its shipping configuration.

Three observations on the shipped pair, so nobody reads a candidate's figure
against a shipped-pair figure that was only ever one sample:

- **The shipped body-yaw pair does not always clear step 3's worst bound.** Over
  three tours the body yaw worst was **0.4024** / 0.3884 / 0.3275 rad, and step 3
  passes a class at a worst of at most 0.4 rad; 190015Z's reading is a joint
  behind its generator (predicted 0.4132 rad moving +0.0227 rad/period, present
  0.0107 rad). The class stands past that bound about one tour in three — five
  further tours read 0.3818 / 0.4035 / 0.4139 / 0.3978 rad — and that is the
  class's own spread, not a reading of a pair: the instrument reads the body yaw
  **gain-bound at exactly the shipped acceleration**, so the loop and not the
  motor is what holds it back, and no candidate inside
  `CAPABILITY_PLATEAU_RATIO` of `(20, 50)` is a candidate at all. The bound
  judges a class whose pair is moving, and the yaw's is not; these readings are
  recorded as samples and bake nothing. Where a yaw window does reach a check is
  the yaw rows of any fixture cut from a tour, under the 1.5 × sizing
  assertion.
- **The shipped antenna pair clears the bound on every tour on record** —
  0.3795 / 0.3782 / 0.3855 rad on the `500 / 0 / 100` gains of those nights,
  half again the worst 0.578; the same pair on the vendor's `200 / 0 / 0` reads
  0.3913 rad, which is where the antenna pin now comes from and is still under
  the bound — **and the recorded head worst is not the worst the pair has
  done.**
  `RECORDED_WORST_HEAD_RESIDUAL_RAD = 0.3884` is 210529Z's figure; 190015Z's
  body yaw reached 0.4024 rad, and 0.4024 × 1.5 = 0.6036 is past the 0.6 rad
  screen — a headroom of 1.491 rather than 1.5. The constant is not moved: it is
  re-baked from a candidate's confirmation tour by cutting fixtures, which the
  baseline does not do.
- **These figures are the depth-two walk, and one of them fell.** Against the
  depth-three record every class cell moved by at most a period of travel at
  `50` units (0.024 rad): eight up, and 190015Z's antenna down, 0.4018 →
  0.3795. Its depth-three worst was a right antenna *ahead* of its prediction
  on a reversal (present 6.2786 rad, predicted 6.6804 rad with the generator
  moving −0.0210 rad/period) and the model now reaches that reversal a period
  sooner, so the same sample reads 0.3779; its depth-two worst is a different
  sample, the left antenna *behind* its prediction at 0.3795. Both are the same
  one-period shift of the model's clock. A future re-walk that moves a class by
  clearly more than a period of travel, either way, is a finding and not a
  figure.

### Antenna hold — probes

`hold-probe` on both antennas at the **shipped** `500 / 0 / 100`; `--gains` was
never given. Three probes per antenna, six in all, twelve phases, at the default
three seconds per phase — phase A reads and rewrites the goal at the driver's
cadence, phase B reads only. Both antennas hung limp pointing down throughout
(servo 18 resting at 4026 counts, servo 17 at 62), because the probe commands no
angle. Series fetched to `.local/records/` by `make bench-fetch`.

The probe refused on arrival: servo 18 had its Bus Watchdog at 10, left armed by
an earlier session. `make bench-run ARGS="off"` did **not** clear it — the next
probe refused again at 10 — and `make bench-run ARGS="reboot"` did. That is the
reading the `hold-probe` bullet above now states.

Achieved sample rate 2363–2469 Hz, about 7100 readings in phase A and 7400 in
phase B, against the ~1 kHz the instrument was budgeted for. Servo 18, bound 2
counts **[A]**:

| series | phase | readings @ rate | p-p | reversals/s | interval mean (spread) | dominant period (regularity) |
|---|---|---|---|---|---|---|
| `hold-probe-1788831126-18.csv` | rewrites | 7117 @ 2373 Hz | 1 | 27.7 | 83.34 (183.55) | 97 samples, 40.88 ms, 24 Hz (0.06) |
| | reads alone | 7383 @ 2461 Hz | 1 | 13.0 | 158.66 (289.76) | 1967 samples, 799.29 ms, 1 Hz (0.10) |
| `hold-probe-1788831140-18.csv` | rewrites | 7124 @ 2375 Hz | 1 | 7.7 | 307.09 (526.02) | 104 samples, 43.79 ms, 23 Hz (0.08) |
| | reads alone | 7379 @ 2460 Hz | 1 | 7.7 | 313.05 (539.70) | 10 samples, 4.07 ms, 246 Hz (0.08) |
| `hold-probe-1788831148-18.csv` | rewrites | 7108 @ 2370 Hz | 1 | 9.7 | 216.11 (445.89) | 39 samples, 16.46 ms, 61 Hz (0.07) |
| | reads alone | 7408 @ 2469 Hz | 1 | 10.3 | 197.43 (382.20) | 1209 samples, 489.62 ms, 2 Hz (0.09) |

Servo 17 **[A]**: `hold-probe-1788831162-17.csv` rewrites 7089 @ 2363 Hz, 1
count, 2.7 reversals/s, interval mean 423.57 (spread 732.20), dominant period
2880 samples / 1218.58 ms / 1 Hz (0.22); its reads-alone phase **0 counts, no
reversal**; `…169-17.csv` and `…177-17.csv` **0 counts in both phases**, no
interval and no period. Samples off the modal encoder count run 0–2 % of each
phase.

Three readings come out of this, and they are why step 1 is a gain sweep on the
machine rather than more probing:

- **No hunt pointing down, at the gain that hunts elsewhere.** Every phase is
  single-LSB dither: intervals whose spread is 1.5–2.4 × their own mean, and an
  autocorrelation regularity of 0.06–0.10 (0.22 once, over 2.7 reversals/s).
  The limit cycle recorded at the rest pose reads ±5 counts, an interval mean of
  1.96 with a spread of 0.29, and an autocorrelation of ±0.96. The dominant
  periods in the table are the largest lag of a series with no periodic
  structure and **are not a frequency measurement of anything**. Read narrowly:
  the joint was hanging where the servo found it, never driven to a goal, and
  the step probes later hunted that same direction at 7–8 counts once a goal
  step had put the joint there. What this phase shows is that the pose alone,
  under no commanded arrival, does not cycle.
- **The 50 Hz goal rewrite is not indicted.** Phase A and phase B are the same,
  so the "phase A hunts, phase B is quiet" branch was never entered and the
  driver keeps rewriting every setpoint every period.
- **The hunt is pose-dependent**, and a probe that moves nothing cannot reach
  it. No high-rate instrument is needed for a 10–13 Hz phenomenon on a 50 Hz
  grid — the grid resolves it at four to five samples per cycle — so the motion
  stack is the instrument for it, and no "move then hold" probe was added to the
  bench.

### Antenna hold — the vendor rung, three motion runs

Overlay `cogs/servo_gains.textproto` only: antennas `200 / 0 / 0`, legs
`800 / 100 / 300`, body yaw `200 / 0 / 0`, sha256 `30ee8fac…39c1`. Profile the
shipped `20 / 50`, detector armed. Three `make motion-run`s of the wake gesture,
35.6 s and about 1782 pose samples each: `motion-log-20260908T013751Z`,
`…T013949Z`, `…T014057Z`.

Two antenna holds per run — the pre-raise stow hold at about +6.1 s and the
raised rest hold at about +14.9 s. Bound 2.0 counts (0.0031 rad) **[A]**:

| run | hold | p-p | reversals/s | period line |
|---|---|---|---|---|
| 1 | right +6.10 s, 200 rdg | 0.0 | 0.0 | no period |
| 1 | left +6.10 s, 200 rdg | 1.0 | 0.3 | no period |
| 1 | right +15.08 s, 151 rdg | 1.0 | 12.0 | ≈ 7.7 samples, 6.5 Hz apparent, spread 3.7 |
| 1 | left +14.90 s, 160 rdg | 1.0 | 1.6 | ≈ 41.0 samples, 1.2 Hz apparent, spread 19.8 |
| 2 | right +6.12 s, 199 rdg | 0.0 | 0.0 | no period |
| 2 | left +6.12 s, 199 rdg | 0.0 | 0.0 | no period |
| 2 | right +15.06 s, 152 rdg | 1.0 | 18.5 | ≈ 5.2 samples, 9.5 Hz apparent, spread 3.3 |
| 2 | left +14.90 s, 160 rdg | 1.0 | 8.5 | ≈ 7.9 samples, 6.3 Hz apparent, spread 8.1 |
| 3 | right +6.10 s, 200 rdg | 0.0 | 0.0 | no period |
| 3 | left +6.10 s, 200 rdg | 0.0 | 0.0 | no period |
| 3 | right +15.08 s, 151 rdg | 1.0 | 22.3 | ≈ 4.4 samples, 11.3 Hz apparent, spread 1.9 |
| 3 | left +14.90 s, 160 rdg | 0.0 | 0.0 | no period |

Twelve holds, every one inside the bound, and every period line carrying the
dither signature — a spread comparable to or larger than its own mean — rather
than the limit cycle's.

The same gesture, same setpoint, at the shipped `500 / 0 / 100`
(`motion-log-20260907T184533Z`, the reference run) **[A]**: left antenna at
+14.90 s, 161 readings, **9.0 counts (0.0138 rad) peak to peak, 25.3
reversals/s, period ≈ 3.9 samples, 12.7 Hz apparent, spread 0.3**. That hold is
`trace-antenna-hunt.csv`; run 3's left antenna hold at +14.90 s, 0.0 counts, is
`trace-antenna-still.csv`.

The cost of the softer loop, read and not judged **[A]**: the antenna's worst lag
behind its goal grows from 2.6573 rad at the shipped triple to 2.7168–2.7350 rad
at the vendor's, while it arrives *closer* to stow (0.0004–0.0020 rad from it,
against 0.0042–0.0081) and its worst residual against the model *falls*
(0.0870–0.0900 rad against 0.1013). A loop that follows its generator less
tightly is a loop the generator describes more easily.

Health over the three runs **[A]**: 7.30–7.60 V, peaks 29–35 C, rise ≤ 1 C over
a 36 s run, worst error byte `0x01` on all nine, worst cycle 5.04 ms against the
20 ms grid with no window over.

These runs are what the still fixture was cut from. What they do **not** contain
is a hard stop, or the fold: the arrival is the shipped profile's, about 20
counts per period, and the only pose the watch judged here is the rest pose. So
they are not the evidence `cogs/servo_gains.textproto`'s antenna comment and
`DEFAULT_GAINS.antennas` carry — that is the step-probe sweep below, which
judged the fold and found it hunting at every gain and pair tried.

### Head rows

No head row is judged. `cogs/stillness_report.rs` judges the antennas and folds
every head row into one unjudged worst hold, so the readings here are notes a
person reads, recorded because a reader meeting a 4-count leg line for the first
time needs to know what it is and is not.

Over the nineteen motion runs on record **[A]**, at leg gains no session has ever
moved (`800 / 100 / 300`): **leg 6 reads 3.0–4.0 counts and leg 1 reads 2.0–3.0
counts on the raised hold in essentially every run, at every configuration
including the shipped one** — the reference run at the shipped gains reads leg 1
at 3.0 and leg 6 at 3.0. Most of those excursions carry 0.0 reversals/s and a
mean error that opens near −0.008 rad and averages −0.003 to −0.004 rad over the
hold: a loaded crank drawn onto its goal by the integral term across the window,
which a peak-to-peak spread of present position reads as movement. That is a
recorded reading, not a rung verdict and not a fault; no bound is added for a
head row and none is widened.

One outlier is the reading a person looks at: `motion-log-20260908T013949Z`
(session 2 run 2), **leg 2 at 8.0 counts, 2.5 reversals/s, period ≈ 5.1 samples,
spread 2.0** — the only head row on record whose period line looks like a
regular interval rather than dither. It has not recurred in any other run.

The fold's caveat, for the same reason: an excursion figure alone does not
separate a hunt from a loaded joint closing on its goal. The reversal rate, the
interval spread and the mean error do.

### Antenna hold — read from every kept log

Every hold below was read from a `//cogs:trace_export` of the run named, one row
per 20 ms driver period. A *hold* is a run of periods over which the joint's goal
did not change; the *judged tail* skips the first 0.5–1 s after the goal last
moved. *Head still* means every head goal — the body yaw and the six legs — was
constant over that tail. Excursion is peak-to-peak present position in counts.
The method reproduces the analyzer's own reading of the reference run's rest hold
(9–11 counts, 12.7 Hz), which is what validates it.

The antenna angles are extended-position: `±3.32` is stow, pointing down and
leaning 10.2° inboard; `±6.458` is the rest pose, ten degrees off vertical;
`±9.2 … ±9.8` is a turn past stow, pointing down again. The runs in the table
below were flown at the vendor's stow angle, `±3.05` — 5.2° short of straight
down on the outboard side — which is what the readings marked `stow` are of.

| gain | head | pose | excursion, counts | character | runs |
|---|---|---|---|---|---|
| `500 / 0 / 100` | still | rest | left 9–11, right 0 | 12.6–12.7 Hz, autocorrelation 0.77–0.95 | reference run **[A]**; both pre-campaign tours **[S]** |
| `200 / 0 / 0` | still | rest | 0–1 both antennas | no period line | 3 wake runs **[A]**, 15 wake runs **[A]**, both capability tours **[S]** |
| `500 / 0 / 100` | still | stow | 0–1, and one 10 | the 10 is the right antenna's second second after arriving | reference run **[A]** |
| `200 / 0 / 0` | still | stow | 0–1 | no period line | 18 wake runs **[A]**, both capability tours **[S]** |
| either | **moving** | rest | 7–24 both antennas | irregular, ~10 Hz, autocorrelation ≈ 0.35 | all four tours **[S]** |

Twenty head-still runs at `200 / 0 / 0`, eighteen of them analyzer-read, with no
exception — including the two capability tours, whose head-still rest holds read
left 0 and 1, right 1 and 1 after seven minutes of motion at the servos' own
limits. At `500 / 0 / 100` the left antenna hunts in every head-still rest hold
on record and the right antenna is quiet in every one.

Four readings the table does not carry:

- **The sway under a moving head is not the loop [S].** In all four tours every
  rest hold during which the head was in motion reads 7–24 counts peak to peak
  on *both* antennas, in the same windows, at `200 / 0 / 0` and at
  `500 / 0 / 100` alike — for instance a 10.6 s antenna hold under a 0.6 rad head
  move at t ≈ 190 s: left 24 / right 15 at `200`, left 16 / right 10 at `500`.
  The single head-still rest hold in the same tours is 0–1 at `200` and 11 at
  `500`. It is the rod following the platform it is mounted on, it is not a
  defect, and it is what a stillness verdict pointed at a tour without a
  head-still qualifier would misread. That is why the tour report's stillness
  section judges only the holds no head row was commanded somewhere new across,
  and prints the rest with their figures and no verdict: a tour holds the
  antennas at rest almost only while the head moves.
- **The pose that hunts, in this table, is the rest pose [S] [A].** Settled stow
  holds are quiet at both gains here — in the tours, in the wake runs, and in
  the 2.4 kHz probes. The mechanism is backlash unloaded near vertical, which is
  what the rest pose's ten-degree lean was chosen against; the lean alone was
  not enough at `500 / 0 / 100`, and with the vendor's gain it is. **Read this
  table against the sweep below**: every stow reading in it is of the vendor's
  `±3.05` fold reached softly or under head motion, and the step probes then
  hunted that fold at 7–8 counts at every gain and every pair tried. The stow
  holds above are quiet because of how they were arrived at and which window the
  watch judged, not because the fold is a quiet pose.
- **No head-still hard stop into the rest pose exists in the record [S].** The
  wake runs arrive at the `20 / 50` profile, about 20 counts per period
  (1.2 rad/s). The capability tours' fast antenna arrivals — 200–270 counts per
  period, 15–19 rad/s — all landed either at pointing-down poses (quiet, 0–1
  counts even at 13 s) or during head motion. The two tours' one head-still rest
  hold each was entered slowly: the goal last moved by 0.02 rad while each joint
  stood 0.035 rad from it, and the approach ran at 0.025–0.028 rad per period
  tapering to zero over the last 0.2 s — a clip's decelerating tail, with the
  `(32767, 1620)` generator nowhere near binding. So the hypothesis that a fast
  move stopped hard is what starts the hunt was untested at any gain in this
  table. **The step probes tested it, and the answer is no**: at the same triple
  the down pose hunts at 7–8 counts in the same 10–13 Hz band at
  `(522, 640)`, `(261, 640)`, `(130, 640)`, `(65, 640)` and `(20, 50)` alike,
  while the sides pose is quiet at all five. The arrival's hardness moved the
  settle travel by seconds and the excursion by nothing; the pose selects the
  hunt, and the answer to it was to lean the fold (the sweep's record below).
- **The vendor gain's cost is a parking deadband [S].** Under
  `tour-log-20260908T020318Z` the antenna parks 0.10–0.13 rad short of a goal it
  is not moving toward, over 47 samples — for example the right antenna at
  −5.794 rad against goals of −5.673 to −5.722 rad at 23.4–23.5 s. That is a
  proportional deadband under friction: not a stillness reading (the excursion
  is zero — the joint is not moving) and not a tracking one (the screen is
  0.6 rad), but a parking error a person can see, and the thing an integral term
  exists to close. The `0.10–0.13 rad` figure is **[S]** and stays here, keyed to
  its tour; no code cites it. **No hold reproduces it**: the rest hold's mean
  error is inside the two-count bound at every hold arrival on record, hard or
  soft, so this is a resting joint whose goal moved by less than its breakaway
  error and not a deficiency a hold can read. The analyzer-read parking figure
  the record does hold is the right antenna's 0.026 rad at the sides pose on the
  `sides←down` arrival (the sweep below); the instrument for the content reading
  is a tour-side parking reading over the tour's zero-travel chasing samples,
  which `TODO(antenna-hold-gains)` carries as work rather than as a figure.

### Body yaw — gain ladder

A P-only ladder on the body yaw at the shipped `20 / 50` profile, detector armed,
antennas at `200 / 0 / 0`, legs untouched. Fifteen `make motion-run`s of the wake
gesture, about 35.7 s and 1783 samples each: six at `400 / 0 / 0`
(`motion-log-20260908T023518Z`, `…T023717Z`, `…T023809Z`, `…T024332Z`,
`…T024418Z`, `…T024504Z`, overlay sha256 `605bda80…ac07`), six at `800 / 0 / 0`
(`…T023900Z`, `…T023947Z`, `…T024033Z`, `…T024613Z`, `…T024658Z`, `…T024744Z`,
sha256 `c50da1ad…6ffb2`) and three at the vendor's `200 / 0 / 0`
(`…T024856Z`, `…T024942Z`, `…T025029Z`, sha256 `30ee8fac…39c1`). Three per rung
was the design's ask; the extra three per rung and the three fresh baseline runs
were flown because the first three did not separate the rungs.

The gesture leaves the yaw one of three holds depending on whether the raise
moved it — a 3.98 s stow hold, an 8.48 s raised hold, or a 17.26–17.28 s stow
hold over 864–865 readings — and only the long hold shows the hunt, on about half
the runs. Widest yaw hold per run, bound 2.0 counts **[A]**:

| P | short holds | long holds |
|---|---|---|
| 200 | 0.0, 1.0, 1.0, 1.0, 1.0 | 2.0 — 4.1 reversals/s, spread 26.0 (dither) |
| 400 | 0.0, 0.0, 2.0 (20.4 reversals/s, spread 2.4) | **3.0, 3.0, 3.0** — 11.8–15.8 reversals/s, period ≈ 4.7–6.3 samples, 8.0–10.7 Hz apparent, spread 2.2–3.6 (limit cycle, three of three) |
| 800 | 0.0, 1.0, 1.0 (14.2 reversals/s, spread 5.3) | 2.0 (spread 10.6), 2.0 (spread 11.6), **3.0** — 12.7 reversals/s, period ≈ 5.9 samples, 8.4 Hz apparent, spread 2.9 (limit cycle, one of three) |

Nothing else in the gesture separates the rungs: the yaw never chases (`no
sample found this class chasing a setpoint more than 0.1 rad away`, all fifteen
runs), its worst residual is 0.0015–0.0150 rad against the 0.6 rad screen, and
its arrival and lag figures are identical to three decimals across the rungs.
Health flat at every rung: 7.30–7.60 V, peaks 29–35 C, rise ≤ 1 C, worst cycle
4.6–5.3 ms.

**Outcome: both rungs hunt and the ladder closes at the vendor's `200 / 0 / 0`.**
Nothing was committed for the yaw; the class takes step 2's gain-bound rule. Two
readings go with that:

- **The ladder is non-monotonic.** `800` is quieter than `400` on every
  comparison these runs hold. That is recorded, and it does not promote `800`:
  a rung on which the machine showed a limit cycle hunts, and the bound is not
  widened to accept one.
- **The next lever is a derivative term at a P above the vendor's**, carried as
  `TODO(body-yaw-gains)`. Its value is a guess today, between the antennas' 10
  and the legs' 300, and the diagnostic long hold arrives on only about half the
  runs, so it needs its own instrument choice: `hold-probe <yaw id> --gains
  P,I,D` for the probe half and a motion run that produces the long hold every
  time for the confirmation half.

### Capability

Two tours, one overlay, read by the same instrument. Overlay
`.local/experiment/step2-capability/cogs/`: `servo_profile.textproto` legs
`(32767, 445)`, body yaw `(32767, 445)`, antennas `(32767, 1620)` (sha256
`70396767…0f85c`); `mover_params.textproto` `tracking_armed: false`
(`579a24c8…3161`); `servo_gains.textproto` antennas `200 / 0 / 0`, the rest the
tree's (`30ee8fac…39c1`). `tour-log-20260908T020318Z` (the kept tour, run
directory `1788832560362471129`, 435 s, 21745 pose samples) and
`tour-log-20260908T031159Z` (the repeat, `1788836681242224325`, 21739 samples).
Both played **69 of 69 motions of the library whole**, with 0 skipped cycles, no
wire failure, no dropped goal, no tick fault and no obstruction report; worst
deviation from stow 0.0215 and 0.0230 rad.

**`(32767, 445)` and `(32767, 1620)` are writable on this hardware.** No register
refused either pair on any of the nine servos, on either tour.

Both tours report one finding — "this run's tracking detector was disarmed … the
run measures capability and is not a run that can pass" — which is the designed
verdict for a capability run, not a defect. The residual figures are meaningless
by construction, the model having been the wrong one on purpose; for the record
they are, at the measured dead time of two samples **[A]**, body yaw 0.3812 /
0.3729 worst, legs 0.3738 / 0.3526, antennas **0.7323 / 0.7976** — the antennas
past the 0.6 rad screen, which is exactly what a generator written above the
motor does to the model and why no armed run is ever flown at this overlay. The
campaign read the antennas at 0.3216 / 0.3313 on the same tours before the dead
time was measured; a period of the model's clock at this overlay is a period of
travel at 0.777 rad, so a one-period shift moves an antenna figure by about that
much, and the two readings are the same recording under two clocks.

**Health: no class's candidate is bounded by it.** Rise 1–2 C on the kept tour
and 0–2 C on the repeat, peaks 29–36 C with servo 18 the hottest, 7.20–7.60 V,
worst error byte `0x01` on all nine, no servo within 14 C of
`TEMPERATURE_STOP_C = 50`. A tour at the servos' own limits heats this machine no
more than a tour at the shipped pair does.

Chasing travel per period, in Profile Velocity units, kept → repeat **[A]**:

| class | chasing | p10 | p50 | p90 | max | Velocity Limit |
|---|---|---|---|---|---|---|
| body yaw | 960 → 960 | 19 → 19 | 35 → 35 | 74 → 70 | 106 → 109 | 445 |
| legs | 1165 → 1205 | 54 → 54 | 125 → 125 | 281 → 275 | 438 → 429 | 445 |
| antennas | 2861 → 2837 | 102 → 106 | 205 → 205 | 397 → 393 | 1033 → 972 | 1620 |

**Travel against the error it was made at**, band median in Profile Velocity
units with the band's sample count, kept → repeat **[A]**:

| error band, rad | body yaw | legs | antennas |
|---|---|---|---|
| 0.10–0.20 | 26 (562) → 26 (575) | 106 (785) → 102 (830) | 144 (885) → 144 (865) |
| 0.20–0.30 | 48 (254) → 48 (240) | 179 (236) → 173 (231) | 211 (1289) → 208 (1289) |
| 0.30–0.40 | 70 (120) → 70 (120) | 326 (91) → 326 (89) | 278 (319) → 272 (314) |
| 0.40–0.50 | 86 (24) → 86 (25) | 342 (34) → 352 (37) | 355 (170) → 352 (172) |
| 0.50–0.60 | — | unread, n 19 → n 18 | 406 (55) → 422 (51) |
| 0.60–0.70 | — | — | 617 (49) → 611 (50) |
| 0.70–0.80 | — | — | 710 (34) → 688 (37) |
| 0.80–0.90 | — | — | 707 (30) → 710 (32) |
| 0.90–1.00 | — | — | 640 (27) → 707 (25) |

No band in either tour fell away from the band under it: every top band either
rose or read one speed with its neighbour, so no band was read past and all
three regimes are read over the whole readable table.

- **body yaw: gain-bound on both tours.** The fastest band read stands more than
  `CAPABILITY_PLATEAU_RATIO` above the band below it all the way up, so the speed
  was still rising with the error when the content ran out: this class was held
  back by its own loop, not by its motor, at the vendor's `P 200`. Its candidate
  velocity is the `[0.2, 0.3)` band's median, **48 units**.
- **legs: motor-bound on both tours**, plateau 0.30–0.50 rad over two bands,
  slowest median in it **326 units** on both.
- **antennas: motor-bound on both tours**, plateau 0.70–1.00 rad over three bands
  on the kept tour and 0.60–1.00 over four on the repeat, slowest median in it
  **640 → 611 units** — a ratio of 1.047, inside the repeatability gate.

**Goal steps and the ramp median**, kept → repeat **[A]**:

| class | steps | ramp p50 | ramp p90 |
|---|---|---|---|
| body yaw | 1 → 0 | none — under `CAPABILITY_STEP_MIN_SAMPLES` | — |
| legs | 24 → 25 | **287 → 307** units | 410 → 420 |
| antennas | 18 → 16 | **522 → 522** units | 584 → 635 |

The listing is what the dead time is read off, and the reading is unambiguous:
every step's first post-write period carries a few units of travel and its second
carries several times that. On the kept tour, `-0.3048 rad` written to the right
antenna at `1788832829800000000` reads `16, 198, 381, 617, 764, 828` units over
the six periods after the write; `+0.2325 rad` on leg 5 reads `13, 128, 288, 361,
387, 438`. Across the two tours all 75 goal steps move in the period after the
write and ramp in the one after that, and the second period exceeds the first in
every one. One period is the driver's read-before-write and the second is the
servo starting late in the period after the write; the third period that a
fitted depth of three charged to the servo was the generator's own ramp at
`a = 20`.

**The increase — a chasing period travelling at least a count further than the
period before — is not the acceleration**, and the two windows are recorded so
that the difference is on the page **[A]**:

| class | increase p50, this instrument | over a three-goal window | goal-step ramp p50 |
|---|---|---|---|
| body yaw | 20 → 20 | 20 → 20 | none |
| legs | 92 → 92 | 123 → 113 | 287 → 307 |
| antennas | 123 → 123 | 225 → 225 | 522 → 522 |

The middle column is the figure the campaign first proposed; it was computed
against the three-period dead-time window the tree carried at the time, in the
same ruling that measured the window to be two, and the tree's instrument reading
the same recordings over the measured window reads the left column. Either way
the increase sits at about a third of the ramp the same joints show on a goal
step: a pair written from it would ramp the legs in nine periods where the motor
does it in three, and a slow model is followed easily, so the motor-bound
acceleration is read off the steps.

**The candidates** these tours produce, which are what `RECORDED_CAPABILITY_*`
carries: legs `(287, 326)`, antennas `(522, 640)`, body yaw `(20, 48)`,
acceleration first. Of the three, only the legs' was confirmed and committed;
the body yaw's is inside the instrument's own ratio of the shipped pair and so
was never a candidate, and the antennas' is on record and uncommissioned (the
`### Confirmation` record below). Repeatability, worst ratio
across the two tours: legs 1.07 on the acceleration and equality on the velocity;
antennas equality on the acceleration and 1.05 on the velocity; body yaw equality
on both. All inside the 1.15 gate.

One divergence is recorded rather than reconciled: the campaign's own offline
script read the antennas' step ramp as **532** over 13 goal steps on the kept tour
and 15 on the repeat, where this instrument finds 18 and 16 steps and reads
**522** on both. The two readings differ by 1.02, inside the repeatability ratio,
and the figure the tree bakes is the instrument's, because that is the one a
re-read reproduces.

The XL330 has no acceleration-limit register: address 40 is unmapped and reads 0
on all nine servos. Profile Acceleration is bounded by its own range, 0 to
32767, and 0 disables the profile's acceleration stage.

### Antenna gain sweep — the hard stop

Thirty-six `make motion-probe` runs on the unit, payload built at `ca24ae1`, the
detector armed from the tree's `cogs/mover_params.textproto` in every one — no
overlay carries that file. Six rungs of six runs, three of
`probe/antenna-step-a` and three of `probe/antenna-step-b`. Gains outside the
antennas are the tree's in every overlay (legs `800 / 100 / 300`, body yaw
`200 / 0 / 0`); only the antennas' triple and pair move.

Every figure is the run's own `library_tour_report` stillness section under the
probe-run standard **[A]**. The head stands bit-for-bit at the raised base
through every probe hold, so every hold below is head-still and judged. The four
holds of a probe run are the engagement hold (where the driver took hold at
arm-on; nothing drove the joint there) and the clip's three poses — sides at
`ANTENNA_OUTBOARD`, down at `STOW_ANTENNAS`, up at `NEUTRAL_ANTENNAS` — each
reached by a one-frame goal step and held 6.5 s.

| rung | overlay | antennas | verdict |
|---|---|---|---|
| A0 | `stepA-a0-p200-522-640` | `200 / 0 / 0` at `(522, 640)` | *Hunts*, 5 of 6 |
| A0 ×½ | `stepA-a0-a261-640` | `200 / 0 / 0` at `(261, 640)` | *Hunts*, 6 of 6 |
| A0 ×¼ | `stepA-a0-a130-640` | `200 / 0 / 0` at `(130, 640)` | *Hunts*, 5 of 6 |
| A0 ×⅛ | `stepA-a0-a65-640` | `200 / 0 / 0` at `(65, 640)` | *Hunts*, 6 of 6 |
| A0 floor | `step1b-antennas-200-0-0` | `200 / 0 / 0` at the tree's `(20, 50)` | *Hunts*, 4 of 6 |
| A1 `P 400` | `stepA-a1-p400-2050` | `400 / 0 / 0` at `(20, 50)` | *Hunts*, 6 of 6 |

**Twelve earlier runs, kept for what they are.** Before the probes existed the
same ladder was walked over the wake gesture at `(522, 640)` — four rungs of
three `make motion-run`s, payload `71993b1`, overlays
`stepA-a0-p200-522-640`, `stepA-a1-p400-522-640`, `stepA-a1-p300-522-640`,
`stepA-a2-p200-i50-522-640` — and the gesture is min-jerk at every profile pair,
so those runs were never a hard stop and the fold was never judged in them (its
hold is 2.8 s against the watch's 6 s). Their A0 *Quiet* verdict is void. What
stands from them is the **rest pose's** own proportional bound **[A]**: at
`P 300` and `P 400` the left antenna hunts the rest hold at 7–9 and 12 counts,
11.1–12.0 Hz, spread 0.3–0.4, three of three at both rungs, where `P 200` reads
0–1 counts; and what the stiffer loop bought, the antennas' worst residual
behind their own generator, falls monotonically over the same range —
0.2132–0.2393 rad at `200`, 0.1551–0.1575 at `300`, 0.1128–0.1137 at `400`. The
`I 50` rung is the only one on record with any residual *ahead* (0.0147–0.0195
rad, the integrator's overshoot) where every `I 0` rung reads 0.0000. Health was
flat in all twelve (servo 17 peak 32–33 C, servo 18 34–35 C) and no run faulted.

**Every hunting hold is the same reading.** The **down** pose, the left antenna
at 7–8 counts against the 2.0 count bound, reversing 20–23 times a second at an
apparent period of 4.4–4.9 samples (10.2–11.3 Hz) with a spread of 0.4–0.5 — a
limit cycle, in the same 10–13 Hz band as every antenna hunt on record. The
right antenna joins it intermittently (3–7 counts at 0–9 reversals/s, which is a
one-sided drift inside a hold and not the cycle). At `P 400` both antennas hunt
the down pose at 8–10 counts and the up pose joins in at 14–15 counts on the
left, 5 of 6. The **sides** pose is quiet at every rung (0–1 counts) and the
**engagement** hold is quiet in all thirty-six.

No `TickFaults` row and no obstruction report in any of the thirty-six runs, so
no rung is *Faulted*. Health flat throughout: servo 17 peak 33–34 C, servo 18
peak 36 C against `TEMPERATURE_STOP_C` 50, rail 7.40–7.60 V, worst error byte
`0x01` and nothing else.

**What the ladder bought.** Antennas' worst signed residual behind their own
generator, per rung, 0.0000 rad ahead in every run (`I 0` throughout), against
the 0.6 rad screen: 0.4830–0.5232 at `(522, 640)`, 0.3762–0.4427 at
`(261, 640)`, 0.2577–0.3341 at `(130, 640)`, 0.1879–0.2454 at `(65, 640)`,
0.0463–0.0706 at `(20, 50)` and 0.0173–0.0376 at `(20, 50)` with `P 400`. The
residual falls with the pair, as the model says it should: at `(522, 640)` the
generator asks for 0.307 rad a period and the joint is most of a period behind
it. The top three rungs sit over the 0.4 rad bound step 3 sets per class, and
the probes' step is not step 3's stimulus.

**The settle allowance says the stimulus really varied**, printed and never
judged: the left antenna's down-pose arrival reads 2.99 rad peak to peak inside
the allowance at `(522, 640)`, 2.89 rad at `(65, 640)` and 4.27 rad at
`(20, 50)`, where the servo trails the streamed step by seconds. Its reversal
reading is already the cycle at every rung with a hunting hold (19–21
reversals/s, period 4.5–4.9 samples), so the joint is cycling before the judged
window opens.

Three readings the table does not carry:

- **The pose selects the hunt, not the arrival.** The excursion is 7–8 counts at
  every pair from the motor's ceiling to the shipped floor while the arrival
  time moved by seconds. Whether the hardness changes the *odds* — 4 of 6 at
  the floor against 5–6 of 6 above it — six runs a rung cannot say.
- **The engagement hold is the same orientation, quiet.** At `∓3.04…3.09` the
  antennas hold the same physical direction as the down pose, one turn back in
  extended position, and read 0–1 counts in all thirty-six runs. Nothing drove
  the joint there. So the record has the orientation quiet where it was never
  commanded and hunting where it was — but the two differ in both winding and
  in whether a goal was chased, and this sweep separates neither.
- **The right antenna parks 0.026 rad short at the sides pose** on probe b's
  `sides←down` arrival, at `(522, 640)` and `(20, 50)` alike (mean error
  −0.0254 to −0.0261 rad over a hold whose excursion is 0–1 counts) **[A]**.
  Sixteen to seventeen counts of steady offset on a still joint: a parking error
  of the kind the vendor gain's cost bullet above describes, now analyzer-read,
  at a pose and an arrival direction no ladder had named.

**Outcome: the antenna triple stays at the vendor's `200 / 0 / 0` and the gain
question is closed**, with the down-pose hunt recorded as a measured behaviour
of this machine rather than an open gain question. No triple tried is quiet at
every pose: `200 / 0 / 0` hunts one pose, `400 / 0 / 0` hunts two, the shipped
`500 / 0 / 100` hunts the rest pose. `P 200` is the proportional bound,
confirmed twice — the rest pose hunts at `300` and `400` under the wake gesture
and the up pose hunts at `400` under the probes, and the cycle's amplitude grows
with P at both poses that hunt. A downward P rung was not walked: it trades the
one visible cost on record, the parking error, which scales as `1/P`, for a hunt
that has been accepted.

**What answered the hunt was the pose, not a gain.** `STOW_ANTENNAS` now leans
10.2° inboard of straight down instead of 5.2° outboard of it, on the same
mechanism the rest pose's lean was chosen against. Six `make motion-probe` runs
at the leaned fold, payload `edc1cc8`, under A0's own overlay
`stepA-a0-p200-522-640` — the stimulus A0 ran, so the only thing that moved is
the constant: `probe-log-20260909T014542Z` and `…T014804Z`, `…T014858Z`,
`…T014957Z`, `…T015053Z`, `…T015151Z`.

- **The down pose is quiet in 6 of 6** — every judged hold in every run 0–1
  counts, every report exit `0` — where the unleaned rung hunted it at 7–8
  counts and 10.2–10.4 Hz in 5 of 6 **[A]**. The 1-count readings are 0–1
  reversals a second, the encoder's own flicker, not the cycle. Run a1's
  down-pose hold is kept as `trace-antenna-fold-still.csv`, so the reading the
  fold was moved on is replayed under `make check`.
- The settle allowance still reads the cycle on the way in: 19.6–26.0
  reversals/s at a period of 3.7–4.7 samples with a spread of 1.1–1.7, over an
  allowance whose peak-to-peak is the arrival's own travel. The lean did not
  stop the antenna cycling as it arrives; it stopped the cycle persisting into
  the hold, which is what the bound judges.
- The sides-pose parking error is unmoved (right antenna −0.0247 to −0.0261 rad
  on the `sides←down` arrival, left +0.0123), and the down pose parks within
  0.0012 rad.
- **The new fold against the folded head**: every run ends in the disarm stow at
  the leaned pose with the head folded. `nothing unread at the release` in all
  six, worst deviation from stow 0.0059–0.0096 rad against
  `DEFAULT_STOW_TOLERANCE` 2° (0.0349 rad), and both antennas 0.0005 rad from
  stow in every run with no trend across the six, so `at_stow` is true six
  times. A rod resting short against the fold would read as a deviation toward
  the goal's near side; the reading is 0.0005 rad on both rows. These six runs
  were made with nobody at the machine, so contact was not checked by sight; the
  disarm deviation is the instrument that stood in for it.
- Antennas' worst residual 0.4722–0.5247 rad behind, the same band as the
  unleaned rung's; health flat (servo 17 peak 33–34 C, servo 18 35–36 C).

What stays open is the parking half, carried by `TODO(antenna-hold-gains)`: the
integral term at `P 200` against the sides-pose offset, with the probes as the
instrument for the arrival reading and a tour-side parking reading for the
content one.

### Confirmation

Two configurations were flown: three candidate-overlay tours that walked the
design's × 0.8 ladder and stopped it, then one tour and one motion run at the
configuration that ships. Payload `edc1cc8` throughout, detector armed from the
tree's `cogs/mover_params.textproto`, every overlay carrying
`cogs/servo_profile.textproto` alone so the tree's gains fall through
(gains digest `05c534b57d180d0f…` in every run). No gain moved in any of them.

**The ladder, and why it was retired.** Worst signed residual behind, and p99.9,
per class per iteration, in rad **[A]**:

| iteration | antennas pair | antennas worst / p99.9 | behind, in periods | body yaw pair | body yaw worst / p99.9 |
|---|---|---|---|---|---|
| 0, `tour-log-20260909T015956Z` | `(522, 640)`, 0.306955 rad/period | **0.5078** / 0.4458 | 1.45 | `(20, 48)`, 0.023022 | 0.3818 / 0.3464 |
| 1, `…T020934Z` | `(418, 512)`, 0.245564 | **0.4238** / 0.3707 | 1.51 | `(20, 48)` | **0.4035** / 0.3533 |
| 2, `…T021726Z` | `(334, 410)`, 0.196643 | **0.5989** / 0.3813 | 1.94 | `(16, 38)`, 0.018225 | **0.4139** / 0.2880 |

The legs held `(287, 326)` through all three and passed every time (0.3273 /
0.3242 / 0.3288 rad, p99.9 0.1651 / 0.1674 / 0.1650), motor-bound each time with
the plateau inside 1.02 of the candidate. The ladder never touched them.

The antennas' reading is not what the "joint behind" branch models. That branch
reads a joint behind as a trapezoid faster than the motor under load and answers
with a slower trapezoid. The capability re-read says the opposite: at
`(522, 640)` and `(418, 512)` the class is motor-bound with a plateau *above*
the commissioned velocity (675 against 640, 534 against 512) — the servo reaches
its generator's speed. What it does not do is stand where the generator stands.
Its p99.9 sits 1.45 and 1.51 periods of travel behind at those two rungs, the
same figure to within 4% at two speeds, which is a **position loop lagging a
fast ramp** — the servo's own following lag at the tree's `P 200`, not a motor
short of its trapezoid. `plant.rs`'s header lists what the model leaves out and
a following lag on a fast ramp is on that list under no name, so the finding is
for the plant model and not for a margin.

A slower pair therefore shrinks the lag in radians while the lag in periods
stays, down to a floor no pair moves: the first step did what the branch says
(0.5078 → 0.4238, ratio 0.835 against the pair's 0.80) and the second reversed
it. At `(334, 410)` the p99.9 stopped falling (0.3707 → 0.3813, which is where
the three `20 / 50` tours put the antennas' *worst*), the worst went to a single
sample at 0.5989 rad — 1.57 × its own tour's p99.9 where the earlier rungs read
1.14 — and the tour read the class **gain-bound** where both earlier rungs read
it motor-bound, so the confirmation could no longer read the motor through the
pair. The failing window moves every iteration (`toc-toc-toc`,
`sharp_side_tilt`, `simple_nod`). The × 0.8 ladder is retired on this record:
no third rung was run and no class takes a × 0.8 step in a later run.

The body yaw was never a candidate. `(20, 48)` is inside
`CAPABILITY_PLATEAU_RATIO` of the shipped `(20, 50)` — 50/48 is 1.04 — on a
class the offline baseline read gain-bound at exactly the shipped acceleration,
so its 0.3818 and 0.4035 rad are two more samples of the shipped pair's own
spread and the `(16, 38)` rung ran a class that was never failing its pair.
Health was flat over all three tours (servo 18 peak 37 C, servo 17 peak 34 C,
rail 7.10–7.60 V, worst error byte `0x01`), every tour played the library whole
with `0 skipped cycle(s)` in all 69 windows, and no run carried a `TickFaults`
row or an obstruction report. Iterations 1 and 2 got no motion run: the failing
reading is the tour's, and a wake run under a pair the ladder is about to step
past measures nothing.

**The confirmation that shipped.** Overlay
`.local/experiment/confirm-legs-287-326`, `cogs/servo_profile.textproto` alone
with the legs at `(287, 326)` and the body yaw and antennas at the tree's
`(20, 50)` — the configuration that ships if it passes; profile digest
`125bf5fa52849ea5…`.

Tour `tour-log-20260909T100150Z/1788947672638475962`, analyzer exit `0`: `69
script(s) asked for 69 of the sidecar's 69 motion(s)`, every motion played
whole, all 69 windows carrying samples with `0 skipped cycle(s)`, 21718 of 21719
samples judged, no `TickFaults` row and no obstruction report **[A]**.

| class | worst (behind / ahead) | p99.9 | worst window | pair it ran at |
|---|---|---|---|---|
| body yaw | 0.3978 (**0.3978** / 0.3704) | 0.3351 | `pollen/emotions/no_sad1` | `(20, 50)`, the tree's |
| legs | 0.3319 (**0.3319** / 0.2206) | 0.1653 | `pollen/dances/grid_snap`, leg 3 | `(287, 326)`, the candidate |
| antennas | 0.3913 (**0.3913** / 0.3055) | 0.3465 | `pollen/dances/stumble_and_recover`, right antenna | `(20, 50)`, the tree's |

- **Legs, the one candidate: pass.** 0.3319 rad behind against the 0.4 rad
  bound, a fourth sample beside the ladder's 0.3242–0.3288, and the capability
  re-read has the class motor-bound with the plateau's slowest median at **326
  units** against the candidate's 326 — a ratio of 1.00 and the fourth tour to
  read it motor-bound at this pair (333, 294, 291 on the ladder's tours).
- **Body yaw, a control**: 0.3978 rad, inside the 0.3275–0.4035 rad the class
  has read over five tours at a pair the instrument cannot tell from `(20, 50)`
  — a sixth sample of that spread, baking nothing.
- **Antennas, a control with no record at this configuration**: 0.3913 rad is
  the first tour reading of the antennas at the tree's `200 / 0 / 0` on
  `(20, 50)`, and it clears the 0.4 rad margin the shipping configuration has to
  clear. It is where `RECORDED_WORST_ANTENNA_RESIDUAL_RAD` now comes from.

**The legs' goal-step ramp median is the content's, not the motor's**: 82 units
over 37 steps, where the wide-open capability tour read 287 over 24. The steps
are the library's own 0.10–0.23 rad leg moves at a commissioned acceleration of
287, so the generator was never the binding constraint on them — the same reason
a shipped-pair class's capability line is recorded and not judged. The velocity
half of the re-read reproduces the recorded 326 to the digit, so
`RECORDED_CAPABILITY_LEGS` keeps the wide-open tour's pair and its comment
carries both halves. The body yaw read gain-bound at `(20, 50)` (bands 19 / 29 /
26 / 32 / 38 / 42 / 48 units) and the antennas motor-bound at a plateau of 45
units — the generator's own cap, read through a generator written far below the
motor, which is why neither figure is judged against a recorded capability. No
band fell away and no class read content-bound anywhere in the tour.

Stillness, printed and not gating **[A]**: the only head-still judged antenna
holds are the engagement holds at the leaned fold, `∓3.3195`, 0.0 counts on both
rows with a mean error of ±0.0000 rad. Every other antenna hold is `under head
motion, not judged` and reads 7–20 counts at 7.7–12.2 Hz, which is the sway this
record describes for a tour. Head rows are a note: the body yaw's widest hold is
44.0 counts over 56.60 s at a spread of 56.7 samples, drift and not a cycle, and
the legs read 2–3 counts. Health flat over seven minutes: hottest servo 18 at
35 C, rail 7.20–7.60 V, worst error byte `0x01`, and `0 windows whose worst
cycle ran past the 20.000 ms grid`.

Motion run `motion-log-20260909T100358Z/1788948199579603367`, the wake gesture
at the same overlay, analyzer exit `0`: the gesture whole, 1781 of 1781 samples
judged, worst residual body yaw 0.0215 rad behind, legs 0.1497 behind, antennas
0.0464 behind and 0.0631 ahead; reached upright and stowed. Both antenna holds
inside the bound (0–1 counts). The disarm at the leaned fold with the head
folded: `nothing unread at the release`, worst deviation 0.0199 rad inside the
2° tolerance, both antennas 0.0005 rad from stow — the same figure the six
leaned-pose probe runs read — and `torque off on 9 rows, 9 of them read back`.

**Verdict: pass, and one pair is committed.** The legs go to `(287, 326)` in
`cogs/servo_profile.textproto` and `SHIPPED_PROFILES`; the tour's two worst
windows are kept as `trace-tour-grid-snap.csv` and
`trace-tour-stumble-and-recover.csv`, the first mixed-pair fixtures in the
suite; `RECORDED_WORST_ANTENNA_RESIDUAL_RAD` is re-baked to 0.3913 rad at the
gains the class ships, `RECORDED_P999_LEGS_RESIDUAL_RAD` to the ladder's three
tours at the commissioned pair, and the obstruction-cost figures in
`docs/fault-management.md` and `CLAUDE.md` are re-derived at the legs' pair with
the regime each is evaluated under. The body yaw stays at `(20, 50)`, off the
candidate list. **The antennas' measured capability `(522, 640)` is on record
and not commissioned**: at the tree's `P 200` the class follows a fast generator
1.45–1.94 periods of travel behind, and the sizing rule the 0.6 rad screen is
derived from has no room for that lag (0.5078 × 1.5 = 0.762). What stands
between the measured pair and the tree is a plant model that accounts for the
following lag, which is `TODO(session-servo-profile)`'s next step; the kept tour
logs at all three antenna pairs are that work's data, and no hardware run is
owed before it.

**Next step after that**: `TODO(antenna-raise-clock)`. The wake raise is one
0.8 s min-jerk duration for the head and the antennas together, so the pair the
model cycle commissions changes the antennas' *follow* and not the clock; a
raise the user would call a snap is the antennas' raise on its own clock, read
at the pair once it is settled.
