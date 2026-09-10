# Recorded runs

Per-period traces written against the unit by the bench loop of the day, kept
as test data. Each file holds one bench session's runs, appended in the order they
were commanded; a run is a move, and the files here predate per-file run
numbering, so every run in them is numbered `0` and they are told apart by the
period counter starting again.

They are here because the guards this crate ships — step bounds, the tracking
threshold, the antennas' separation — are sized against what this machine
actually did, and a measurement nothing replays is folklore by the next release.
`tests/replay_test.rs` replays them against the values the cog path runs on;
one test per file says what that file is kept for and asserts it, so a fixture
cannot quietly rot into a file nobody checks.

The `profile` column is the servo trajectory profile the run was recorded
under, as the commissioning sweep writes it — acceleration then velocity in
register units, per servo class as legs / body yaw / antennas — and the grid
the loop was driven at. Both matter: what the tracking comparison judges a
joint against is the trajectory its own class's two registers define, stepped
once per period, so a run is only replayable against the plant it was actually
made on, which is why the column is read per class. The bench nights and the
earlier tours each wrote one pair into all nine servos, and those rows say so
once; the two confirmation-tour rows carry the legs at the pair the tree
commissions them at and the other two classes at `20 / 50`, which is the
profile the deployment ships, the fast-pair row carries the antennas at the
capability their own motor reads and the body yaw at the 48 that morning's
sweep left in the register, the leaned-fold row carries the antennas at
the measured capability the gain sweep ran them at, and the two probe rows carry
every class at what the tree ships, which is what makes them the confirmation.
The bench nights ran a faster profile and a
slower loop than the machine ships; `Run::period_ns` reads each run's grid back
out of its own timestamps, and the replay suite builds the plant per run from
that.

The `gains` column is the position loop each class was running — the three
classes in the order legs, body yaw, antennas, each as P / I / D — and it is
there because the model carries a following lag. A lag is a reading of one loop at one gains triple, so a row
is replayed at the lag read for *its* triple: the legs' and the antennas' where
the tree has one, and zero — the generator alone — where a class's triple has no
reading or where the column says `—` because no triple is on record for it. A
recording judged at another loop's lag is the same mistake as one judged at
another pair's generator, and the suite states each row's three lags beside its
three pairs for that reason.

| file | runs | profile | gains | what it records |
|---|---|---|---|---|
| `trace-verify2.csv` | 1 | 400 / 600 all three, 32 ms | —, —, 500 / 0 / 100 | the validated gesture: head and both antennas up in 0.82 s, measurably arrived before the last goal went out |
| `trace-fast4.csv` | 1 | 400 / 600 all three, 32 ms | —, —, 500 / 0 / 100 | the antenna speed record — 187° in 0.40 s, 855°/s peak — on a pair running two clocks |
| `trace-newgains.csv` | 2 | 400 / 600 all three, 20 ms | changed mid-file, —, 500 / 0 / 100 | the same step command either side of the leg gain change: ~4° of permanent droop on the loaded pair before, about a degree after, and an antenna pair under the pace floor before and over it after |
| `trace-stagger.csv` | 3 | 400 / 600 all three, 24 and 32 ms | —, —, 500 / 0 / 100 | the tip-to-tip collision, run 2: both antennas stall at mirrored angles for over 40 periods — about 1.06 s — then spring apart |
| `trace-tour-toc-toc-toc.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the clip library's longest travel and worst goal lag on record — 2.96 rad behind on an antenna — at a residual under 0.1 rad: the standing case that a lag says nothing about health |
| `trace-tour-side-peekaboo.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the library's worst leg reversal excursion, 0.2691 rad on leg 3 under the legs' own following lag |
| `trace-tour-proud1.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the library's worst antenna sample of all, 0.3782 rad, on the `500 / 0 / 100` antenna gains the class no longer runs — judged at the generator alone, because no lag has been read on that loop, which is why it stands above `stumble_and_recover`'s reading at the loop the class ships |
| `trace-tour-no-sad1.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the worst head residual any kept fixture holds, a body yaw reversal at 0.3884 rad — the class runs the same pair and the same gains it did that night, so this is still the figure the head pin carries. The worst body yaw on record is 0.4024 rad, which no fixture holds |
| `trace-wake-20260906.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the shipped wake gesture and the 36 s hold after it, at the profile the deployment commissions |
| `trace-antenna-hunt.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 500 / 0 / 100 | the antenna hunt, 2026-09-07: the raise and the hold after it, over which the left antenna swings 9 counts about its held goal on a period of about four samples and the right sits inside one count at the same reversal rate |
| `trace-antenna-still.csv` | 1 | 20 / 50 all three, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the quiet hold, 2026-09-08 at the vendor's `200 / 0 / 0` gains: the same raise and hold with the left antenna reading the same encoder value every period of the judged tail and the right inside its usual count — the recording the two-count bound is asserted to pass |
| `trace-antenna-fold-still.csv` | 1 | 20 / 50 legs and body yaw, 522 / 640 antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the quiet hold at the leaned fold, 2026-09-09 at the vendor's `200 / 0 / 0` gains: both rods stepped onto `∓9.6032` rad — the Minimum Risk Condition's fold, a turn up — and holding inside one count, where the same rung pointing straight down hunted 7–8 counts at 10.2–10.4 Hz. The reading the stow angle was moved on |
| `trace-tour-grid-snap.csv` | 1 | 287 / 326 legs, 20 / 50 body yaw and antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the commissioned leg pair confirmed: the six cranks' worst residual on a whole-library tour at it, 0.1983 rad on leg 3 under the legs' own following lag, against the 0.4 rad bound that tour was read against |
| `trace-tour-stumble-and-recover.csv` | 1 | 287 / 326 legs, 20 / 50 body yaw and antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the same tour's worst antenna reversal, 0.3625 rad on the right antenna, at the vendor's `200 / 0 / 0` antenna gains and under the 1.2 periods of following lag read on that loop — the class at a pair it does not ship, kept because the aggregate assertion needs every fixture's antenna worst beside the pin |
| `trace-tour-sharp-side-tilt-fast.csv` | 1 | 287 / 326 legs, 20 / 48 body yaw, 522 / 640 antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the antennas at their commissioned capability: the library's worst antenna window at that pair, 0.2536 rad on the right antenna under the same 1.2 periods of following lag, against 0.5078 rad at the generator alone — the antenna pin, confirmed on the machine by the probe runs beside it |
| `trace-probe-antenna-sweep.csv` | 1 | 287 / 326 legs, 20 / 50 body yaw, 522 / 640 antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the residual instrument of the armed confirmation, 2026-09-09: the library's antenna stress in miniature — streamed ramps at the pair's own cap, a reversal at an arrival, a reversal mid-move, the outboard arc — worst 0.2105 rad on an antenna. The shape a following lag is largest under, at the whole configuration the tree ships |
| `trace-probe-antenna-step-a.csv` | 1 | 287 / 326 legs, 20 / 50 body yaw, 522 / 640 antennas, 20 ms | 800 / 100 / 300, 200 / 0 / 0, 200 / 0 / 0 | the largest antenna sample of the six armed confirmation runs, 0.2535 rad: a one-frame goal step of more than a radian, where the generator ramps from rest and the shaft follows a lag behind it. The shape the sweep leaves out |

The four bench files were drawn from an archive with its plots that is not
tracked. The thirteen later ones are windows of kept run logs, cut by
`//cogs:trace_export` from nominal instants: the seven tour files are the clips
the library's worst residuals were measured on, printed per motion by the
library tour's own report, the wake file is that gesture's whole log, and the
two probe files are the worst antenna windows of the armed runs that confirmed
the antennas' pair. Four of the seven tours ran one pair everywhere; two are the
tour that confirmed the legs' pair and one is the tour that flew the antennas at
their own. Those three and the two probe files are the fixtures here whose three
classes do not carry the same pair. The
three antenna-hold files are cut differently and for a different reader — they
are judged by the stillness watch rather than the tracking comparison, so each
begins before the last goal write onto the held pose and ends in the first
ticks of the step off it, which is what lets the shipped settle allowance be
spent inside the file and the window open where the live watch opened it. Two
of the three are the same raise at two gains triples, which is why those two
are read together; the third is a different pose at the second of those
triples, which is what makes it the fold's own reading and not the raise's.

Adding a file here means adding the test that says why it is worth keeping.
