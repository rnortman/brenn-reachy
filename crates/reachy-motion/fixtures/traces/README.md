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
register units — and the grid the loop was driven at. Both matter: what the
tracking comparison judges a joint against is the trajectory those two
registers define, stepped once per period, so a run is only replayable against
the plant it was actually made on. The bench nights ran a faster profile and a
slower loop than the machine ships; `Run::period_ns` reads each run's grid back
out of its own timestamps, and the replay suite builds the plant per run from
that.

| file | runs | profile | what it records |
|---|---|---|---|
| `trace-verify2.csv` | 1 | 400 / 600, 32 ms | the validated gesture: head and both antennas up in 0.82 s, measurably arrived before the last goal went out |
| `trace-fast4.csv` | 1 | 400 / 600, 32 ms | the antenna speed record — 187° in 0.40 s, 855°/s peak — on a pair running two clocks |
| `trace-newgains.csv` | 2 | 400 / 600, 20 ms | the same step command either side of the leg gain change: ~4° of permanent droop on the loaded pair before, about a degree after, and an antenna pair under the pace floor before and over it after |
| `trace-stagger.csv` | 3 | 400 / 600, 24 and 32 ms | the tip-to-tip collision, run 2: both antennas stall at mirrored angles for over 40 periods — about 1.06 s — then spring apart |
| `trace-tour-toc-toc-toc.csv` | 1 | 20 / 50, 20 ms | the clip library's longest travel and worst goal lag on record — 2.96 rad behind on an antenna — at a residual under 0.1 rad: the standing case that a lag says nothing about health |
| `trace-tour-side-peekaboo.csv` | 1 | 20 / 50, 20 ms | the library's worst leg reversal excursion, 0.274 rad on leg 3 |
| `trace-tour-proud1.csv` | 1 | 20 / 50, 20 ms | the library's worst antenna reversal excursion, 0.354 rad |
| `trace-tour-no-sad1.csv` | 1 | 20 / 50, 20 ms | the worst residual on record, a body yaw reversal at 0.368 rad — the figure `threshold_rad` carries its margin over |
| `trace-wake-20260906.csv` | 1 | 20 / 50, 20 ms | the shipped wake gesture and the 36 s hold after it, at the profile the deployment commissions |

The four bench files were drawn from an archive with its plots that is not
tracked. The five later ones are windows of the kept 2026-09-06 run logs, cut
by `//cogs:trace_export` from the nominal instants the library tour's own
report prints per motion; the four tour files are the clips the library's worst
residuals were measured on and the wake file is the whole gesture log.

Adding a file here means adding the test that says why it is worth keeping.
