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

| file | runs | what it records |
|---|---|---|
| `trace-verify2.csv` | 1 | the validated gesture: head and both antennas up in 0.82 s, measurably arrived before the last goal went out |
| `trace-fast4.csv` | 1 | the antenna speed record — 187° in 0.40 s, 855°/s peak — on a pair running two clocks |
| `trace-newgains.csv` | 2 | the same step command either side of the leg gain change: ~4° of permanent droop on the loaded pair before, about a degree after |
| `trace-stagger.csv` | 3 | the tip-to-tip collision, run 2: both antennas stall at mirrored angles for over 40 periods — about 1.06 s — then spring apart |

The whole archive these were drawn from, with its plots, is not tracked. Adding
a file here means adding the test that says why it is worth keeping.
