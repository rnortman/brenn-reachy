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
  Per class: how many samples the joint spent chasing a setpoint it was behind,
  the per-period travel over them (p10/p50/p90/max, in rad and in Profile
  Velocity units) against the recorded Velocity Limit, and the ramp — the
  largest single-period gain in travel as a joint sets off — in Profile
  Acceleration units. Per servo: first, last and peak temperature with when,
  voltage range, and the worst error byte the run latched. Notes, not verdicts.
- **The stillness watch's period line**, in the `stillness` section: mean and
  spread of the sample count between reversals, and the apparent frequency at
  the window's own sample rate. A regular limit cycle reads as a small spread; a
  bit of encoder dither reads as a spread comparable to its mean. The apparent
  frequency is an alias — at a series sampled at `r` Hz an apparent `f` is any
  of `|r·k ± f|`.
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
  quiet phase this probe exists to find. `off`, or a reboot, clears both.
- **The register sweep's Acceleration Limit** (`make bench-selftest`), checked
  against the data sheet's 32767 and allowed to fail. It is the ceiling a
  Profile Acceleration is written under; a write past it is refused by the servo
  and fails the commission loudly.

## The procedure

Each step names its pass rule before it runs. Every accepted figure is committed
as configuration or as a pinned constant, with its run directory recorded below.
**An unexpected reading goes to a person before anything is made green.**

### 0. Offline baseline

**Done** — the figures are under "The record" below, and both are baked in
`cogs/pose_reading.rs`.

No hardware. `library_tour_report` over the kept tours and `first_motion_report`
over the kept motion runs, with the tree's three configuration files staged as
each log's `config/` — that is what those runs ran on. Two figures came out that
nothing had measured before:

- **Temperature**: per servo, the peak a healthy tour reaches and how far in.
  `TEMPERATURE_STOP_C` is set between that peak and the servo's own 70 °C
  shutdown, with the margin stated against the recorded figure, and is a `fail`
  rule in `health_summary`.
- **Residual distribution**: per class, worst and p99.9 under the shipped pair.
  The worst was already pinned; the p99.9 is `RECORDED_P999_*_RESIDUAL_RAD`,
  three figures per class rather than one, and every residual line prints the
  range.

### 1. The antenna hold

**Step A — probe the mechanism.** `hold-probe` on each antenna, three times
each, at the shipped gains. Read peak-to-peak, the reversal intervals, the
dominant period and its regularity per phase — a limit cycle turns round on a
near-constant interval and correlates strongly at one lag, while encoder dither
turns round as often at scattered intervals and correlates at none.

- *Phase A hunts, phase B is quiet*: the goal rewrite drives it. Establish first
  whether reads alone hold the servo's Bus Watchdog off (`bench-run
  ARGS="watchdog"`, its reads-only phase). If they do, the fix is a driver
  change — rewrite only a changed setpoint — and is its own piece of work. If
  they do not, the rewrite stays and gains are the lever.
- *Both phases hunt at the same frequency*: a loop or a mechanical limit cycle.
  Repeat at `--gains 200,0,0`, then `150,0,0`, `100,0,0`, one probe each per
  antenna, recording frequency and amplitude against P. A frequency that falls
  with P is the position loop's limit cycle; one that does not move while the
  amplitude persists is mechanical. Either way the candidate for step B is the
  lowest P that quietens it.
- *The probe never hunts pointing down* over three tries: the hunt is
  pose-dependent, which a probe that moves nothing cannot reach. Step B starts
  at the vendor's `200,0,0`.

**Step B — confirm on the machine.** With the candidate gains in an overlay,
`make motion-run` three times. **Pass**: every antenna hold in every run inside
the stillness bound. The apparent-period line is recorded either way. **Fail**:
the next rung down — P by 50 to a floor of 100 at D 0, then D 10 at the best P —
three runs per rung. On pass, commit the gains to `cogs/servo_gains.textproto`
and `DEFAULT_GAINS`, rewrite their comments with the run's figures, cut a
still-antenna trace fixture from a passing run, and record the runs below.

The bound is never widened. If no rung on the ladder quietens the antennas and
the hunt is not write-driven, the remaining levers are outside this procedure —
current-based position mode, feedforward gains, or resting the antennas
de-torqued — and that choice is the user's.

### 2. Motor capability

`make bench-selftest` first, so the Acceleration Limit is a recorded figure.
Then one overlay: `servo_profile.textproto` with each class at its two recorded
limits (the head's velocity 445, the antennas' 1620, acceleration at the
recorded limit), `tracking_armed: false`, and step 1's gains. Then
`make library-run`, **attended, with the space clear**.

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
2. **Capability, per class.** The chasing-sample count says which case a class is
   in, and the report names it.
   - *Saturating*: the motor fell short of the generator. Candidate velocity is
     the **p10** of chasing travel in register units, rounded down;
     acceleration the **p50** of ramp increase, rounded down. The low percentile
     is deliberate — the model must be a floor a healthy motor always clears, in
     every direction and under every load.
   - *Content-bound*: the motor executed what the library asked and the
     generator never binds on this content. Candidate velocity is 1.25 × the
     maximum observed per-period travel, acceleration 1.25 × the maximum ramp.
3. Write the three candidate pairs into a second overlay with
   `tracking_armed: true`, and go to step 3.

This run's residual figures are meaningless by construction — the model was the
wrong one on purpose — and its verdict is a `fail` on the disarmed detector. Its
notes are what is read. Its capability figures are baked as
`RECORDED_CAPABILITY_*` constants once a person has read them.

### 3. Confirmation, and the choice

`make library-run` then `make motion-run` under the candidate overlay, armed.
**Pass**: no tick fault, no obstruction report, every window moving, the motion
run's stillness section green, health flat, and per class a worst residual at
most **two thirds of the 0.6 rad tracking screen, 0.4 rad** — the order of
headroom the shipped pair keeps and the margin under which the detector has
never faulted a healthy machine. The p99.9 is printed against the baseline's and
read, not judged: a p99.9 that grew more than the worst did is a motor lagging
everywhere rather than at a few extremes, and is written into the record beside
the pair.

- **Pass**: commit the three pairs to `cogs/servo_profile.textproto` and
  `SHIPPED_PROFILES`, re-bake the recorded worst residuals by cutting this
  tour's worst windows as new fixtures, and re-derive the prose figures that
  quote the old pair's timings.
- **Fail on residual in a class**: the trapezoid at that pair does not describe
  that motor under load. Multiply that class's pair by 0.8, both registers, and
  re-run. Three iterations at most; a class still failing at half its measured
  capability is a motor whose response is not the generator's at any useful
  pace, and the choice — ship the slow pair for that class, or give the model a
  load term — is the user's.
- **Fail on stillness**: re-run step 1B under the new profile before touching
  the profile. Only if no gain on the ladder quietens the antennas under the new
  pair does the antenna pair step down by 0.8.
- **Fail on health**: as step 2's reading 1.

The gates that stand either way: no threshold widened, no bound widened, no
figure baked without a human reading the run.

## The record

Each entry names the log or record directory, the configuration it ran under,
and the figures read out of it. Only the offline baseline has been done.

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
| body yaw | 0.3825 / 0.2570 | 0.3684 / 0.3050 | 0.3110 / 0.2475 | **0.2475–0.3050** |
| legs | 0.2692 / 0.2083 | 0.2738 / 0.1976 | 0.2556 / 0.1925 | **0.1925–0.2083** |
| antennas | **0.4018** / 0.3096 | 0.3542 / 0.2745 | 0.3615 / 0.2962 | **0.2745–0.3096** |

All figures in rad. The already-pinned worsts (0.368 head, 0.354 antennas) are
tour 210529Z's body-yaw and antenna figures, which the analyzer reproduces. The
p99.9 is recorded as a range and not one number because step 3 reads a
candidate's p99.9 for growth, and the run-to-run spread between identically
configured tours — 0.058 on the body yaw, 0.016 on the legs, 0.035 on the
antennas — is the noise floor of that reading. A candidate inside the range has
not grown; one above its top has, by at least the amount above.

Two observations on the shipped pair, so nobody reads a candidate's figure
against a shipped-pair figure that was only ever one sample:

- **The shipped antenna pair does not always clear step 3's worst bound.** Over
  three tours the antenna worst was 0.3542 / 0.3615 / **0.4018** rad, and step 3
  passes a class at a worst of at most 0.4 rad. A candidate antenna pair doing
  exactly what the shipped pair does will therefore fail roughly one tour in
  three and take the × 0.8 step. **The bound stands and a fail is a fail**: it
  is a margin to the 0.6 rad screen, not a statistical claim about the shipped
  pair, and no bound is widened here. The consequence is that the antenna ladder
  is calibrated on the safe side; a marginal fail is not for an operator to read
  as a pass on the day.
- **The shipped body-yaw worst on record is not the worst the pair has done.**
  `RECORDED_WORST_HEAD_RESIDUAL_RAD = 0.368` is 210529Z's figure; 190015Z's body
  yaw reached 0.3825 rad. The screen is sized at half again over the recorded
  worst, and 0.3825 × 1.5 = 0.574 is still under it. The constant is not moved:
  it is re-baked from a candidate's confirmation tour by cutting fixtures, which
  the baseline does not do.

### Acceleration Limit

_Not yet read._ To carry: the nine readings from the register sweep, and whether
they are the data sheet's 32767.

### Antenna hold — probes

_Not yet run._ To carry: per probe, the servo, the gains, both phases'
peak-to-peak, dominant period and regularity, and the CSV's name.

### Antenna hold — confirmation runs

_Not yet run._ To carry: per rung, the gains, the three run directories, each
antenna hold's excursion and period line, and the arrival figures read as the
cost of a softer loop.

### Capability

_Not yet run._ To carry: the tour's log directory, the health readings, and per
class the chasing count, the travel percentiles and the ramp figures in register
units, with the candidate pair derived from them.

### Confirmation

_Not yet run._ To carry: per candidate pair, the tour and motion run
directories, per-class worst and p99.9 residual against the screen, the
stillness section's verdict, and what was committed.
