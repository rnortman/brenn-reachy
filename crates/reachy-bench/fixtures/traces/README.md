# Recorded runs

Per-period traces written by `reachy-bench --trace` against the unit, kept as
test data. Each file holds one bench session's runs, appended in the order they
were commanded; a run is a move, and the files here predate per-file run
numbering, so every run in them is numbered `0` and they are told apart by the
period counter starting again.

They are here because the guards this repo ships — step bounds, the tracking
threshold, the servo gains, the antenna clocks — are sized against what this
machine actually did, and a measurement nothing replays is folklore by the next
release. `trace::metrics` reads them; the tests in that module say what each
file is kept for and assert it, so a fixture cannot quietly rot into a file
nobody checks.

| file | runs | what it records |
|---|---|---|
| `trace-verify2.csv` | 1 | the validated gesture: head and both antennas up in 0.82 s, measurably arrived before the last goal went out |
| `trace-fast4.csv` | 1 | the antenna speed record — 187° in 0.40 s, 855°/s peak — on a pair running two clocks |
| `trace-newgains.csv` | 2 | the same step command either side of the leg gain change: ~4° of permanent droop on the loaded pair before, about a degree after |
| `trace-stagger.csv` | 3 | the tip-to-tip collision, run 2: both antennas stall at mirrored angles for over 40 periods — about 1.06 s — then spring apart |

The whole archive these were drawn from, with its plots, is not tracked. Adding
a file here means adding the test that says why it is worth keeping.
