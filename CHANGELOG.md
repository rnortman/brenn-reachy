# Changelog

All notable changes to brenn-reachy are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project aims to adhere to [Semantic Versioning](https://semver.org/spec/v2.0.0/).

## [Unreleased]

Nothing has been released, and nothing here has driven a motor.

### Added

- **A pose recorder for authoring clips by hand.** `make pose-record` runs a
  recording session: the servos de-torqued, the operator's hands on the head,
  and three streams on one clock --- servo positions at 50 Hz from a new
  read-only bench command (`reachy-bench pose-log`), the speech pipeline
  transcribing spoken labels with each one read back over the speaker, and the
  audio itself. The operator speaks a label, hears it confirmed, then moves the
  head; no screen needed. A streaming motion segmenter
  (`reachy_motion::segments`) classifies the pose stream into still holds and
  motion stretches in real time, so the console says when a pose registered.
  After the session, `pose_session_report` joins the two streams into a
  document: segments with per-joint and head-space figures, utterances with
  their intervals and audio clips, and a timeline an LLM or the operator reads.
  `--extract <segment-id>` writes a clip draft in the format the daemon's
  loader reads, with every channel a delta over the neutral base. A third
  launcher configuration (`robotcpu_record.textproto`) composes the bench, the
  voice host in echo-brain / bypass-wake mode, and the pod --- no driver, no
  control process, no arming path.

- **Servo profiles and gains are per-class and tunable without a code change.**
  The nine servos fall into three classes — the six Stewart-platform legs, the
  body yaw, and the two antennas — each with its own load and its own Velocity
  Limit register. The commissioning sweep now writes each class its own
  acceleration/velocity profile pair and its own PID gains triple, read from two
  checked-in configuration files (`servo_profile.textproto`,
  `servo_gains.textproto`). The plant model and tracking detector follow: one
  trapezoid per class, so a profile tuned to the antennas does not predict the
  legs. All three classes ship at the same conservative pair today; the split is
  the infrastructure the tuning procedure below needs to move them apart.

- **A servo tuning guide and experiment overlay.** `docs/servo-tuning.md` is the
  procedure for measuring what the motors achieve under the library's load,
  choosing a profile pair per class, and confirming the tracking detector still
  works under the new numbers. A new experiment overlay
  (`REACHY_EXPERIMENT_DIR`) lets a push replace the profile, gains, or mover
  parameters without editing the tree: the overlay is named on the console with
  its digest, copied into the run's log alongside the rest of the
  configuration, and both analyzers read the run's own files rather than the
  tree's — so a fetched run carries the numbers that produced it. A disarmed
  detector is always a failed run.

- **Both analyzers measure servo capability.** A new section in each report
  counts the samples each joint spent chasing a moving setpoint, prints the
  per-period travel distribution against the servo's recorded Velocity Limit,
  and reports the ramp — the acceleration the motor managed when it set off from
  rest. The health section now reads the whole series per servo rather than just
  the last sample: voltage range, temperature rise, peak and when, and the worst
  error byte the run latched. A peak reaching 50 °C fails the run; the figure
  is 13 °C over the hottest a healthy library tour has produced and 20 °C under
  the servo's own shutdown. The per-class residual lines print the shipped
  profile's p99.9 from three baseline tours as a range, which is the noise floor
  a candidate's figure is read against.

- **A bench hold probe for single-servo tuning.** `make bench-run
  ARGS="hold-probe <id>"` torques one servo where it stands, writes goals at
  the control rate, records the position series through two phases (with and
  without goal writes), judges the excursion against the stillness watch's
  bound, and writes the series as a CSV the fetch brings home. An operator can
  swap in trial gains before the hold, and the probe restores the originals and
  releases torque on every exit path. The stillness watch's hold-window line
  now carries interval statistics — mean and spread of the time between
  reversals, and the apparent oscillation frequency — so a regular limit cycle
  reads differently from encoder dither.

- **The antennas rest off vertical, and a stillness watch measures whether they
  stay still.** At exactly vertical the gearbox backlash leaves the antenna rod
  balanced on the play with nothing biasing it to one side, and the servo's
  position loop hunts across the gap — the same mechanism the upstream Pollen
  stack documents and shipped a fix for. The neutral antenna pose is now ten
  degrees toward each side's outboard, so the rod's own weight holds the play
  to one side and the loop has something to push against. A new streaming
  stillness watch (`reachy_motion::stillness`) segments every joint's timeline
  into hold windows, measures peak-to-peak excursion and reversal rate in each,
  and judges antennas against a two-count bound that is deliberately written to
  fail before the fix is confirmed on hardware. Both the motion-harness report
  and the speech-run report print a `stillness` section with per-window
  figures; the harness report fails on a hunting antenna, the speech-run report
  notes it. The harness gesture's up-hold is lengthened to eight seconds (from
  two) so the settle allowance and minimum-hold floor fit inside it.

- **Repository scaffolding.** Apache-2.0 license and notice, charter, and TODO
  ledger; secret-scanning commit and push gates wired by `make setup-hooks`; CI
  that independently scans the tree and runs the same `make check` gate a
  developer runs.
- **The Cargo workspace and its five crates**, declared with their dependency
  edges and their reasons for existing: `dxl-proto` (servo wire protocol),
  `reachy-kin` (head kinematics and travel envelope), `reachy-motion`
  (trajectories, per-tick control, arm and disarm sequences), `reachy-bus` (the
  one I/O layer), and `reachy-bench` (bench binary and self-test registry). Each
  is a skeleton at this point — the module headers state the contract each will
  hold to.

- **The robot says its critical alerts out loud.** An alert the host's table
  raises Critical now carries a sentence for a person standing in front of the
  machine, and the voice host speaks it through the speech pipeline where the
  deployment has a voice: the head is not moving, or its motion has stopped, or
  — where de-torquing could not be confirmed — that nobody should touch the
  head. Warnings are never spoken; each latch fires once per run, so a run
  speaks at most three sentences. A host built without a voice narrates and
  raises exactly as before, and every sentence a speaker refuses is an
  `unspoken` line on the console.

- **`reachy_host --check` compares the two names a speech run needs to agree.**
  The new `addressee` conclusion holds the host's own `pod` against the speech
  configuration's `[pods]` table: the pipeline addresses every motion script to
  the connected device's authenticated id, so a host answering to a name that
  is not one of them refuses every script it authors and its head never moves.
  A mismatch is an unheld conclusion, which `make speech-run` refuses on (exit
  11) before anything is provisioned.

- **Offline STT comparison for wake-trim tuning.** `stt_compare` transcribes
  both clips a run's turns export — the whole carve and the carve from the
  wake-trim boundary — through the same recogniser and request the pipeline
  uses, and prints the two readings side by side with a count of turns that
  differ. It runs offline from a fetched run, so measuring whether keeping the
  wake word improves recognition costs no extra on-device time.
  `make speech-run` prints the invocation after fetching records.

- **Async-runtime quarantine gate.** `tokio-quarantine.test.sh` queries the
  build graph for every target that reaches `tokio` and holds the answer to an
  allowlist: the voice host, the filegroups that ship it, and the offline run
  report. A crate on the motion or control path that grows a transitive runtime
  edge now fails `make check` instead of shipping a runtime to a unit.

- **Every payload build says which two brenn-pod revisions it is made of** — the
  revision `MODULE.bazel` pins for the crates the host links, and the one the
  brenn-pod checkout stands at, with the staged audio-device binary's age
  against that checkout's HEAD commit beside it, because the binary is whatever
  the last build there left behind and not the revision the checkout is on now.
  A note, never a refusal: a development checkout legitimately sits ahead of the
  pin, and a payload built out of two revisions otherwise fails on the unit as a
  handshake.

- **Both vendor clip sets — dances and emotions — are imported into the clip
  library and confirmed on hardware.** 69 motions from Pollen's published
  recordings now load, play at their recorded pace, and have human-readable
  names in the sidecar. A new `make library-run` tours every motion in the
  library on the unit in a single unattended session and records servo goals
  and positions throughout, producing the plant-response data the tracking
  detector needs to be re-armed. The importer (`reachy-clip-import`) gained
  automatic still-channel detection, quaternion renormalisation with drift
  reporting, and a per-clip envelope check that refuses frames with no crank
  solution rather than clamping them. Audio sidecars (`.ogg` files shipped
  with the emotions set) are copied alongside their clips.

- **The tracking detector is armed and judges every joint against a plant model
  of the servo's own trajectory generator.** The servos run a trapezoidal
  velocity profile whose two parameters — the velocity cap and the acceleration
  — are the registers the commissioning sweep already writes. A new plant model
  (`reachy_motion::plant`) predicts where each joint should be by stepping that
  profile against the setpoints the driver reported holding, with a two-sample
  dead time measured off the goal steps of two capability tours. A joint whose reading diverges
  from its prediction by more than 0.6 rad for 200 ms is obstructed; a joint
  that is behind but still moving at half the profile velocity or better is slow,
  not stuck, and keeps its window restarting. The servo profile is a single
  checked-in file read by both the session (which writes it to the servos) and
  the motion tick (which models it), and the simulated driver runs the same
  model from its own register cells, so the deterministic suite pins the
  response ladder against a plant that matches the real machine. Five trace
  fixtures cut from the recorded library tour and wake gesture pin the worst
  residual at each profile and assert that a healthy machine never raises. A
  hand held on the head that does not let go is answered in about two seconds:
  the first raise begins a stow, the second raise — when the stow drives into
  the hand — defeats it, and the machine is released where it stands.

- **Content plays without a speed ceiling.** The per-tick step bound and the
  clip speed policy derived from it were sized off two hand-recorded gestures
  and refused seven vendor clips outright. Both are removed: a clip carries no
  ceiling, a composed setpoint is not step-guarded, and the per-tick step bound
  guards only the moves this repo plans for itself.

### Changed

- **A head obstruction during a stow defeats the stow.** Previously a
  `head_obstructed` raise inside a running stow re-commanded the stow into the
  obstruction until the four-second budget expired. The session host now
  recognises that a stow naming no motor to mask cannot be driven through an
  obstruction: the maneuver is concluded as fallen through and the machine is
  released on the next wake. A grab that holds the head for longer than the
  raise latency (~0.7 s) during a wind-down drops it where it is instead of
  folding it, which is the fault-management doctrine's own trade.

- **The simulated driver runs the servo profile, not a per-class slew rate.**
  The three configured slew distances and the transport-delay injection are
  replaced by the plant model stepping each row's profile registers. Every
  arrival instant in the deterministic suite is now an expression over the
  travel that profile needs, derived from the planner's own setpoint stream
  rather than from pre-computed integers. Scenario S14 (new) pins the
  grab-that-does-not-let-go end to end.

- **The clip library asset carries motions, and sequence documents load
  again.** `ClipConfig` gained a derived `max_speed` — the highest invocation
  speed a clip's own frames admit, which a schedule's window is now screened
  against — and `ClipLibraryConfig` gained `motions`: one flattened motion per
  clip, plus one per composed sequence, each a lead gap and a list of
  (clip, speed, hold) segments. **A clip configuration emitted by an earlier
  build refuses to load**, loudly and typed rather than partially; regenerate
  it with `make clip-config`. The name sidecar gained a second table, keyed by
  motion id. **A schedule's overlay windows name motions**, not clips
  (`OverlayWindow.motion_id`), and an overlay plays a whole motion: its lead
  gap holds the base alone, its clips play end to end across their seams, a
  hold freezes the clip before it, and a channel one clip drives and the next
  does not fades out on the outgoing clip's ramp instead of vanishing. Since
  every clip is also a one-segment motion, naming a bare clip costs a schedule
  nothing.

- **Bazel is the only build system.** Every crate is a `rust_library` with its
  tests, `make check` is one lane (`bazel test --config=lint //...`) and CI one
  job, and the device binary is a cross-compile against the hermetic aarch64
  sysroot the pinned Clockwork drop brings — `--platforms=//bazel/platform:reachy-device`
  in place of cargo inside an emulated arm64 container. Nothing in the dev loop
  or on a runner invokes `cargo` or `rustup`, and the pinned `RUST_VERSION` in
  `MODULE.bazel` is the single compiler and the single statement of the edition.

- **rusty-cogs is consumed as it ships.** The two patches this repo applied to
  it — internal linkage for the generated signal trampolines, and dropping the
  root-only `include()` from its `MODULE.bazel` — are fixed upstream, so the pin
  carries no `patches` and `bazel/rusty-cogs-patches/` is gone. `rust_clk_module`
  now takes the repository word, the generation root and the crate name as
  parameters, so `bazel/rust_clk.bzl` is a thin wrapper over
  `@rusty_cogs//bazel:rust_clk.bzl` fixing this tree's repository word and
  naming policy instead of a verbatim copy re-synced by hand at every pin bump.
  With the copy go `bazel/BUILD.bazel`'s `framework_clk_imports` filegroup,
  which the macro now reaches for itself, and `cogs/upstream`'s longhand
  generator invocation, which is a macro call naming `repo` and `crate_name`.

- **A motion script this host authored and then refused is Critical, not a
  Warning.** A body offered to the host's gate now carries where it was authored
  — the pipeline's scripter and the motion harness are local, the bus is remote
  — and a local body the gate refuses means the head will not move for anything
  said to the robot until somebody edits a file. The alert says so once per run,
  under `reachy head refuses its own scripts`, and the refusal line on the
  console carries an `origin` field. A remote sender's refusal keeps the Warning
  it had: the intent channel is not assumed to carry one machine's traffic.

- **The speech-run analyzer opens the channel log and has an opinion about
  motion.** `speech_run_report` read the console alone and counted a dropped
  motion script as a note; it now reads the run's `.olog` beside it and fails a
  run whose scripts the host itself refused, whose session accepted none of what
  the pipeline authored, that the session never took the machine for, or whose
  head never measurably left its first pose. It also fails a run that handed an
  alert to a bus attachment that did not grant alerts, which loses it. A run
  nobody spoke to is still green. Runs that passed before this change can fail
  after it — that is the point of it.

- **The speech-run report prints one line per turn and writes an audio clip for
  each.** Every wake-word activation in a run is now a numbered turn whose line
  shows the transcript, the STT confidence scores (`no_speech`, `logprob`), the
  outcome (dispatched, declined, superseded), and the direction-of-arrival beam
  figure averaged over the turn's audio window. Each turn's raw audio is
  exported as a `.wav` file under `<run>.turns/`, carved from the frame-log
  store the run brought home, so a declined utterance can be listened to
  directly. The summary counts dispatched and declined turns and prints the
  `no_speech` range for each group, which is the reading that says whether audio
  quality degraded across a session.

- **The run report exports the clip the recogniser actually heard.** A turn
  whose records state the wake-trim boundary now gets a second file beside its
  whole carve, `turn-NN.command.wav`, holding the span from that boundary
  onward. The turn line states both offsets: where the wake word ends and where
  transcription began. A wake the listener held back for its command says so on
  the turn line, and a held wake that no command answered is counted apart from
  a wake nobody followed at all.

- **A run's records name the brenn-pod the voice host was built from.** The
  payload's `build-commit.txt` and the `provenance.txt` a fetched run carries
  home gain a `brenn_pod=` line: the pinned revision, or `overlay:<path>` where
  the build resolved the speech crates from a local working tree rather than a
  published revision.

- **Deploy fetches recorded audio alongside console logs.** `make speech-run`
  now brings back the frame-log recording store as `<run>.audio` after each
  speech run, and the preflight check validates that recording is configured
  with a relative store path and reports the per-device and per-pod storage
  caps. A configuration with an absolute recording path is refused.

- **`reachy_host --check` states the STT-confidence gate's thresholds.**
  The speech preflight conclusion now names the `no_speech` and `logprob`
  floors the gate will apply, so an operator can see what the pipeline will
  decline before a run starts. A configuration with no `[stt]` table says so
  rather than dropping the clause silently.

- **brenn-pod pin advanced to pick up the XVF3800 ASR-output routing and
  reliable queue lanes.** `BRENN_POD_REV` moves forward by two published
  commits: the cycle's speech-surface work (chip control registers, startup
  reboot, ASR-output channel routing, per-segment `base_sample`, and STT
  threshold narration) and the earlier reliable-lane queue rework. The host
  crates and the run report compile unchanged against the new surface.

- **A troubleshooting guide for speech degradation.** `docs/speech-degradation.md`
  explains the symptom (utterances declined after the first in a session), how
  to read a run's turn lines and `no_speech` scores, how to listen to a turn's
  clip, what the pod's chip state line says, and the two-session comparison that
  isolates the microphone board's adaptive processing as the cause.

- **Torn console lines are now recovered.** The run report's line classifier
  handles JSON with console text on either side — the shape every real
  transcript line arrives in when the host's console write and the pipeline's
  event write race on the same descriptor. Previously every such line was
  counted as noise and every transcript the tool had ever read was lost.

- **The plant model carries the servo's position loop, and the antennas are
  commissioned at their measured capability.** The model predicted a joint from
  its servo's trajectory generator alone; a proportional position loop follows
  that generator a fixed number of periods of travel behind, which at the 1.2
  periods measured here is three hundredths of a radian at `20 / 50` and 0.368
  rad --- three fifths of the 0.6 rad tracking screen --- at the antennas'
  commissioned pair, spent by a healthy machine doing nothing wrong. The model
  now carries that lag as a measured per-class constant beside the two profile
  registers (`*_following_lag_us`, microseconds so the figure survives a change
  of control grid), read by scanning it over kept recordings for the figure that
  minimises the class's p99.9 residual and accepted only where two recordings at
  different pairs agree, the minimum is a real minimum, and no kept fixture
  reads worse under it. The legs and the antennas read 24 000 microseconds ---
  1.2 periods of the 20 ms grid --- and the body yaw carries none: no recording
  has ever put its motor at its generator's speed, so there is nothing to read
  one off. A lag belongs to the gains triple it was measured at, and every kept
  fixture is now replayed at its own recorded pair *and* its own recorded loop;
  a recording at a triple no lag was read on is replayed at the trapezoid alone,
  which is the honest replay of a loop nobody has measured. The 0.6 rad screen
  is unchanged and the sizing rule is unedited: a better model lowers the worst
  residual and widens the headroom under a fixed screen, and cannot narrow it.
  Every pin on a class that carries a lag falls --- the library's worst leg
  window from 0.3319 to 0.1983 rad, the worst antenna window at the slow pair
  from 0.3913 to 0.3625; the body yaw's 0.3884 and the two antenna fixtures
  recorded on the stiffer `500 / 0 / 100` loop stand, because no lag was read
  for them.

  What that headroom buys is the antennas. They now run `522 / 640` against the
  previous `20 / 50` --- twenty-six times the acceleration and thirteen times
  the velocity --- confirmed by six armed runs at the pair reading a worst
  residual of 0.2535 rad behind and 0.2152 rad ahead against the 0.4 rad bound,
  and sustained 0.1290 rad against the screen.
  Their gains stay the vendor's `200 / 0 / 0`: an integral term was walked at two
  rungs against the parking error it exists for, and both hunt at every pose, so
  the parking error is recorded as the accepted cost. Obstruction-response timing
  follows the pair, as it always has: a held antenna crosses the screen in two
  periods and the pair is torqued off about a quarter of a second after the hand
  lands, on content at the cap and later under a slower goal. No outcome of the
  fault doctrine changes and nothing gates de-torquing.

  The instruments that read all of this ship with it. Both analyzers' residual
  section gains the *sustained* figure --- the largest residual held across a
  whole ten-period window, which is the quantity the detector actually faults on
  --- and a *lag scan* line: the minimising lag over a 0--4 period grid with the
  p99.9 at zero, at the minimum and either side of it, and whether the run is an
  instrument for that class at all. `//cogs:trace_judge` reads an exported
  `//cogs:trace_export` CSV and a run's `config/` directory through the same walk
  and prints the same words as a live report, so a recording written at an older
  state schema can still be read, with `--lag` for a what-if replay. A third
  probe document, `probe/antenna-sweep`, puts the library's antenna stress ---
  saturated moves, a reversal at arrival, a reversal mid-move, the outboard arc
  --- into 15.5 seconds as the residual instrument for a candidate pair when the
  library may not be flown; it is deliberately not a capability instrument, since
  streamed content never opens the tracking error a plateau is read over. All
  three probe documents are now generated by `make clip-config` from a table of
  poses and segments in Rust, held byte-equal to the committed files by a gate in
  both directions, so a retuned pose moves the probes with it.

- **The legs are commissioned at their measured motor capability.** The
  six Stewart-platform legs now run at `287 / 326` (Profile Acceleration /
  Profile Velocity) against the previous `20 / 50`, confirmed by four library
  tours reading 0.3242--0.3319 rad of worst tracking residual against the
  0.4 rad bound. The profile file carries two pairs across three classes: the
  body yaw is gain-bound at exactly the shipped acceleration, so no candidate
  inside the instrument's ratio exists; the antennas' measured capability is on
  record and not commissioned, because at the vendor's gains a fast generator
  outruns the loop by one and a half to two periods of travel and the sizing
  rule has no room for that lag. Obstruction-response timing moves with the
  pair: a hand held on the head is answered in about half a second rather than
  about a second on content that runs the legs at their cap, a brush shorter
  than about 80 ms goes unanswered, and the grace after the first raise is
  160--180 ms. Every one of those is an at-the-cap figure and comes later under
  slower content; no outcome of the doctrine changes and nothing gates
  de-torquing. Two windows of the confirming tour are kept as trace fixtures ---
  the first in the suite recorded with different pairs on different classes ---
  and every fixture is now replayed against the profile it was recorded at.

- **The stow pose folds the antennas 10.2 degrees inboard of straight down**
  rather than 5.2 degrees outboard of it, the vendor's shutdown angle. Straight
  down is the mirror of straight up: gravity loads no side of the gearbox play
  and the position loop hunts across it, which the antennas were watched and
  measured doing at every gain and every profile pair tried. The rest pose has
  leaned against the same mechanism since it was tuned; the fold now leans by
  the same magnitude, and inboard, which also tucks the pair. This moves where
  the machine parks at the Minimum Risk Condition. A trace fixture cut from a
  probe run at the leaned fold asserts the stillness watch passes the pose a
  fault response commands into.

- **The antenna gains are the vendor's `200 / 0 / 0`**, the quietest triple
  a sweep of gain ladders found. The shipped `500 / 0 / 100` hunts the left
  antenna at 9 counts and 12.7 Hz over the rest hold; the vendor's triple is
  quiet at the sides and at the leaned rest pose in every head-still judged
  hold on record, with the proportional bound at 200 --- the rest pose hunts at
  `300` and the pose pointing down hunts at `400`. The pose pointing down
  hunted at 7--8 counts at every profile pair from the motor's ceiling to the
  shipped floor; leaning the fold is what answered that. The body yaw's gains
  were measured and left at the vendor's value, a P-only climb having found
  nothing quieter.

- **The plant model's dead time is two samples, measured rather than fitted.**
  One period is the driver's read-before-write, the second is the servo's start
  late in the period after the write, and every one of the 75 goal steps on two
  capability tours moves in that period and ramps in the next. Every recorded
  residual figure --- the tracking screen's two pinned worsts, the offline
  analyzer's per-class p99.9 ranges, the replay suite's per-fixture pins and
  the record's whole-tour table --- is re-read at the measured depth; the
  0.6 rad screen is unchanged.

- **The capability instrument reads a class against its own tracking error.**
  Both analyzers now cut a class's per-period travel into error bands, name the
  regime the bands read as (motor-bound, gain-bound or content-bound) with the
  plateau the candidate velocity comes off, and list every goal step with the
  ramp of its first whole period of acceleration, whose median is the candidate
  acceleration. A top error band that came out slower than the band under it is
  read past --- that shape is the periods after a large goal step or a stall,
  not a motor losing speed --- and a class left content-bound by such a fall
  offers no candidate. The residual summary says which side of its own
  trajectory a joint stood on, because a joint behind its model and one ahead of
  it call for opposite answers.

- **Two antenna step probes and a single-motion playback target.** The clip
  library gains `probe/antenna-step-a` and `probe/antenna-step-b`: antennas-only
  documents that step to three held poses in one frame each, so the servo's own
  profile generator is the whole of the move and the head stands still through
  it. `make motion-probe MOTION=<name>` plays one motion of the library on its
  own, records under `probe-log-<stamp>`, and both the plan and the verdict come
  off one selection the intent source makes, written into the run directory so a
  fetched run says for itself what it was asked to play. The library tour now
  plays only the content --- every motion whose name does not start `probe/`.

- **The library tour report judges stillness where the head is still.** An
  antenna hold is judged only where no head row was commanded somewhere new
  across it; a hold under head motion is printed with its figures and no
  verdict. Beside every printed hold in both analyzers goes the settle line ---
  what the joint did over the settle allowance the judged window drops, which is
  the arrival and the ring-down after it, printed and never judged.

- **The tuning record is written out.** Every run the campaign flew --- the
  antenna hold probes, the vendor-rung motion runs, the body yaw's gain ladder,
  the antenna gain sweep, both capability tours and the confirmation --- is in
  `docs/servo-tuning.md` with its overlay, its run directories and its figures.
  The procedure it records follows: the antenna step is a gain sweep over a hard
  arrival at every pose, closed at the vendor's triple; the capability step
  reads a class by the regime its error bands fall into; and the confirmation
  step tells a candidate from a control and reads the residual's sign. The
  Acceleration Limit register (address 40, unmapped on the XL330) is removed
  from the vocabulary, the bus table, the provisioning table and the self-test.
  Two fetch-side defects are fixed: a run's configuration copy is moved into
  the run directory it belongs to, and the run directory is found by the
  logger's own stamp rather than by taking the newest directory of any name.

### Removed

- **The Cargo lane.** The workspace manifest, every crate manifest,
  `Cargo.lock`, `rust-toolchain.toml`, the `containers/bench-builder` image
  definition, and the `check-bazel`/`check-commit` split. Third-party versions
  are now stated once as `crate.spec` entries in `MODULE.bazel` and pinned by
  `MODULE.bazel.lock`. `make fix` formats only: `clippy --fix` has no Bazel
  equivalent, and clippy findings are fixed by hand from the gate's output.
  With the manifests go the ways cargo consumes these crates: there is nothing
  for a `git = "..."` dependency or a `cargo publish` to resolve. A consumer
  builds them with Bazel, or pins a revision from before this change.
