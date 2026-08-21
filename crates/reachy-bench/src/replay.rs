//! The bench night, replayed against the guards it sized.
//!
//! Not in the build: this suite drives the retired bench motion layer — the
//! stow-pose targets from `commands.rs` and the trace reader from
//! `trace/metrics.rs` — over the hardware recordings in `fixtures/traces`. The
//! recordings and the guards they size are still live; what stopped compiling
//! is the host that replayed them. Re-pointing this at the cog path is the
//! cutover slice's work.
//!
//! TODO(bench-motion-delete)

/// Every value in `[motion]` is a measurement, and these are the
/// measurements. Two questions, asked of the recordings rather than of
/// arithmetic: does a shipped guard raise anything on a run that went well,
/// and does it catch the one run that did not. A change to a bound, a
/// threshold or a duration that would false-trip the validated gesture — or
/// miss the collision — fails here instead of on the machine.
mod replay {
    use super::*;

    /// The variable naming the directory the hardware trace recordings arrive in.
    /// The test target sets it; nothing else reads it.
    const TRACE_FIXTURES_ENV: &str = "REACHY_BENCH_TRACE_FIXTURES";

    /// A run recorded on real hardware, read back from the fixture of that name.
    ///
    /// The recordings are checked in beside the crate, one file per bench session
    /// and one or more runs per file; their own `README.md` says what each holds.
    /// They are the measurements this machine's guards are sized against, so
    /// replaying them is how a change that would false-trip a validated gesture —
    /// or miss the one collision on record — fails here rather than on the machine.
    ///
    /// The directory is not spelled here: `TRACE_FIXTURES_ENV` carries it, set by
    /// the test target beside the `data` attribute that puts the files in runfiles,
    /// so the two halves stay in one place. Its value is relative to the runfiles
    /// root, which is a test's working directory.
    ///
    /// Panics rather than answers: neither a missing fixture nor a missing
    /// environment is a test case.
    pub(crate) fn trace_fixture(name: &str) -> crate::trace::metrics::Trace {
        let dir = std::env::var(TRACE_FIXTURES_ENV).unwrap_or_else(|_| {
            panic!(
                "{TRACE_FIXTURES_ENV} is unset: the test target has to name the trace fixture \
                 directory beside the data attribute that supplies it"
            )
        });
        let path = PathBuf::from(dir).join(format!("{name}.csv"));
        match crate::trace::metrics::Trace::read(&path) {
            Ok(trace) => trace,
            Err(error) => panic!("the {name} trace fixture reads: {error}"),
        }
    }

    use reachy_motion::{
        JointGroup, JointSet, JointTargets, MotionCommand, TrackingMonitor, Warp,
        dry_pass_peaks, floor_move_clock,
    };

    use crate::commands::stow_pose_targets;
    use crate::trace::metrics::{Run, Sample};

    /// The pose every recorded run started from, with the antennas at the
    /// turn representative the machine was holding them at.
    ///
    /// The legs are the stow pose's own, which is where the recordings
    /// begin to within a degree — asserted below, because the fixtures only
    /// speak for the shipped command while that holds. A degree and not a
    /// count because the servos were holding stow on the gains of that
    /// night, and holding a weight up on a proportional term alone parks a
    /// loaded crank a little short of where it was sent. The antennas come
    /// off the recording because an antenna direction has a representative
    /// per turn, and the sweep the resolver picks depends on which one the
    /// machine stood at.
    fn started_at(run: &Run) -> JointTargets {
        let present = run.samples[0].present.expect("the first period read");
        let stow = stow_pose_targets();
        for (leg, angle) in present.legs.iter().enumerate() {
            let held = crate::testutil::stow_legs()[leg];
            assert!(
                (angle - held).abs() < 1.0_f64.to_radians(),
                "leg {leg} began at {angle}, not the stow pose's {held}"
            );
        }
        JointTargets {
            antennas: present.antennas,
            ..stow
        }
    }

    /// The gesture the recordings are of: stow to neutral, on `durations`.
    fn gesture(run: &Run, durations: MoveDurations) -> (JointTargets, MotionCommand) {
        (
            started_at(run),
            MotionCommand::MoveTo {
                target: JointTargets::default(),
                durations,
                warp: Warp::MinJerk,
            },
        )
    }

    /// The shipped tracking monitor driven over a recorded run, period by
    /// period, answering the joints whose window ran out and when.
    ///
    /// A period whose grouped read fell short is skipped rather than
    /// replayed: the live loop compares a fresh goal only against a fresh
    /// measurement, and a stale one freezes every run where it stands. A
    /// joint the run had released is handed over as masked, which is what
    /// the loop does with it.
    fn trips(cfg: &TrackingFaultConfig, run: &Run) -> Vec<(u64, JointSet)> {
        let mut monitor = TrackingMonitor::new();
        let mut out = Vec::new();
        for sample in &run.samples {
            let Some(present) = sample.present else {
                continue;
            };
            // A released joint is commanded nothing, so it stands at its
            // own angle rather than at a goal it never had.
            let mut goal = present;
            for joint in JointId::ALL {
                if let Some(angle) = sample.goal_of(joint) {
                    goal.set(joint, angle);
                }
            }
            let look = monitor.look(cfg, sample.released(), &present, &goal);
            if !look.exhausted.is_empty() {
                out.push((sample.tick, look.exhausted));
            }
        }
        out
    }

    /// Neither run that went well raises anything in the shipped tracking
    /// monitor.
    ///
    /// The validated gesture and the fastest sweep on record, measured on
    /// the machine, both with every joint following its goal at a distance
    /// the whole way through — 0.245 rad on a loaded leg, 1.38 rad on the
    /// antenna crossing 187° in four tenths of a second. A threshold sized
    /// under those, or a progress minimum over what the machine closes in a
    /// window, shows up here as a fault on a gesture the machine is known
    /// to make well.
    #[test]
    fn the_runs_that_went_well_raise_nothing() {
        let cfg = example_resolved()
            .resolve()
            .expect("the example resolves")
            .motion
            .tracking;
        for name in ["trace-verify2", "trace-fast4"] {
            let trace = trace_fixture(name);
            let run = trace.run(0).expect("the run");
            let trips = trips(&cfg, run);
            assert!(trips.is_empty(), "{name}: {trips:?}");
        }
    }

    /// The one collision on record trips it, on the pair that stalled.
    ///
    /// Both antenna tips met at the crossing and stood there for over forty
    /// periods with the goal three radians away; the head carried on and
    /// arrived. So the monitor has to name the antennas and only the
    /// antennas — which is what decides the response: the pair goes out of
    /// service and the head move finishes, rather than the whole machine
    /// winding down.
    #[test]
    fn the_collision_trips_it_on_the_antennas_and_nothing_else() {
        let cfg = example_resolved()
            .resolve()
            .expect("the example resolves")
            .motion
            .tracking;
        let trace = trace_fixture("trace-stagger");
        let jam = trace.run(2).expect("the failed stow");
        let trips = trips(&cfg, jam);

        assert!(
            !trips.is_empty(),
            "the stalled pair never ran its window out"
        );
        for (at, exhausted) in &trips {
            for joint in exhausted.iter() {
                assert_eq!(
                    joint.group(),
                    JointGroup::Antennas,
                    "a head joint ran its window out at period {at}: {exhausted}"
                );
            }
        }
        // Both sides, and inside one window of each other: they met each
        // other, so neither is the one that failed. Two antennas stalling
        // hundreds of periods apart would be two single-servo faults, which
        // is a different condition with a different answer.
        let ran_out = |side| {
            trips
                .iter()
                .find(|(_, out)| out.contains(side))
                .map(|(at, _)| *at)
        };
        let right = ran_out(JointId::AntennaRight).expect("the right antenna stalled");
        let left = ran_out(JointId::AntennaLeft).expect("the left antenna stalled");
        assert!(
            right.abs_diff(left) <= u64::from(cfg.ticks),
            "the antennas ran their windows out at periods {right} and {left}, further apart \
             than the {} the window itself is: one stalled and the other did not",
            cfg.ticks
        );
    }

    /// The step bounds admit the gestures that were recorded, with the
    /// headroom over their planned peaks that the example's comments claim.
    ///
    /// Against the *plan* and never against the record. The recorded goal
    /// column is what the loop commanded, and these recordings predate the
    /// per-period move clock, so a period that started late sampled the
    /// trajectory further along and commanded a step no planner ever asked
    /// for. The last case below is that inflation, pinned: the fastest
    /// sweep's recorded step is past the bound its own plan clears
    /// comfortably.
    #[test]
    fn the_step_bounds_admit_the_recorded_gestures_with_headroom() {
        let resolved = example_resolved().resolve().expect("the example resolves");
        let cfg = &resolved.motion;
        let tick_hz = f64::from(resolved.tick_hz);

        // The validated gesture, on the clock this file ships, and the same
        // gesture with the staggered antenna pair the file documents —
        // whose quick side is the 0.3 s sweep the speed record was set on.
        let verify2 = trace_fixture("trace-verify2");
        let fast4 = trace_fixture("trace-fast4");
        let cases = [
            (
                "the validated gesture",
                gesture(verify2.run(0).expect("the run"), resolved.up_durations()),
            ),
            (
                "the staggered pair",
                gesture(
                    fast4.run(0).expect("the run"),
                    MoveDurations {
                        head: resolved.up_duration,
                        antennas: [Duration::from_millis(700), Duration::from_millis(300)],
                    },
                ),
            ),
        ];

        let mut planned = Vec::new();
        for (name, (start, command)) in cases {
            let peaks = dry_pass_peaks(cfg, &start, &command, tick_hz)
                .expect("the gesture is measurable");
            planned.push(peaks);
            assert!(
                peaks.legs * 2.0 <= cfg.max_step.legs,
                "{name}: legs plan {:.4} rad against the {:.4} rad bound",
                peaks.legs,
                cfg.max_step.legs
            );
            for (side, peak) in ["right", "left"].into_iter().zip(peaks.antennas) {
                assert!(
                    peak * 1.5 <= cfg.max_step.antennas,
                    "{name}: the {side} antenna plans {peak:.4} rad against the {:.4} rad \
                     bound",
                    cfg.max_step.antennas
                );
            }
            // And no clock needs right-sizing for its span, which is the
            // same statement the shipped durations make about their floors.
            // The pass may still lengthen an antenna to de-phase the pair —
            // the head's clock is what says nothing was floored.
            let stretch = floor_move_clock(cfg, &start, &command, tick_hz).1;
            assert!(
                stretch
                    .is_none_or(|clocks| clocks.dephased
                        && clocks.effective.head == clocks.requested.head),
                "{name}: the shipped clock does not carry it: {stretch:?}"
            );
        }

        // The inflation, pinned. On the quick side the loop commanded half
        // as much again in a period as the planner ever asked for, because
        // the periods it woke on were half as long again as the grid it was
        // sampling. A bound sized to clear the record by the same margin
        // would be half as wide again for no reason the plan gives.
        let quick = planned[1].antennas[1];
        let recorded = fast4
            .run(0)
            .expect("the run")
            .metrics()
            .joint(JointId::AntennaLeft)
            .expect("it swept")
            .peak_goal_step;
        assert!(
            recorded > quick * 1.4,
            "the recorded step {recorded:.4} rad is no longer inflated over the planned \
             {quick:.4} rad, so this case no longer says what it is for"
        );
    }

    /// The separation the resolver holds a pair to admits the clock pair
    /// that swept clean and rejects the one that clashed.
    ///
    /// Both figures come off the recordings' own commanded goals, which is
    /// where a clock pair's phase is visible. The pair that clashed is the
    /// binding end: its widest offset anywhere on the sweep is under the
    /// constant, so nothing phased like that gets through whatever moment
    /// the check happens to land on. The stow that clashed and the raise
    /// that did not are the same two clocks recorded twice — the raise got
    /// through on the odds the debrief measured, about two inboard sweeps
    /// in three, and the check rejects it too.
    #[test]
    fn the_separation_tells_the_clean_pair_from_the_one_that_clashed() {
        // The shipped file's own geometry, so the calibration below is a
        // statement about what an operator's configuration admits and not
        // about a number only the library can see.
        let phase = example_resolved()
            .resolve()
            .expect("the example resolves")
            .motion
            .phase;
        let band = phase.contact_band_rad;
        let stagger = trace_fixture("trace-stagger");
        let fast4 = trace_fixture("trace-fast4");

        let clean = fast4
            .run(0)
            .expect("the run")
            .separation(band, Sample::goal_of)
            .expect("both antennas cross the band");
        assert!(clean.met(phase.separation_rad), "{clean:?}");
        assert!(
            (clean.offset - 0.876).abs() < 5e-3,
            "the validated pair plans {:.4} rad",
            clean.offset
        );

        let clashed = stagger
            .run(2)
            .expect("the stow that clashed")
            .separation(band, Sample::goal_of)
            .expect("both antennas cross the band");
        assert!(!clashed.met(phase.separation_rad), "{clashed:?}");
        assert!(
            (clashed.offset - 0.361).abs() < 5e-3,
            "the pair that clashed planned {:.4} rad",
            clashed.offset
        );
        let widest = stagger
            .run(2)
            .expect("the run")
            .widest_offset(Sample::goal_of)
            .expect("every period of that run carried both antennas");
        assert!(
            widest < phase.separation_rad,
            "the clashing pair reaches {widest:.4} rad, which the shipped separation now admits"
        );

        let survived = stagger
            .run(1)
            .expect("the raise on the same clocks")
            .separation(band, Sample::goal_of)
            .expect("both antennas cross the band");
        assert!(!survived.met(phase.separation_rad), "{survived:?}");

        // And what the tips themselves did, which is what the plan is a
        // proxy for: on the raise they passed the band's edge a third of a
        // radian apart, and on the stow they never reached it at all —
        // they met inside the band and stalled there.
        let tips = stagger
            .run(1)
            .expect("the run")
            .separation(band, Sample::present_of)
            .expect("both antennas cross the band");
        assert!(
            (tips.offset - 0.285).abs() < 5e-3,
            "the tips passed {:.4} rad apart",
            tips.offset
        );
        assert_eq!(
            stagger
                .run(2)
                .expect("the run")
                .separation(band, Sample::present_of),
            None,
            "the jammed pair left the band after all"
        );
    }
}
