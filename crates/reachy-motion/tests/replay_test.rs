//! The bench night, replayed against the guards it sized.
//!
//! Every bound in this crate's shipped configuration is a measurement, and
//! these are the measurements. Two questions, asked of the hardware recordings
//! beside the crate rather than of arithmetic: does a shipped guard raise
//! anything on a run that went well, and does it catch the one run that did
//! not. A change to a step bound, a tracking threshold or the antennas'
//! separation that would false-trip the validated gesture — or miss the
//! collision — fails here instead of on the machine.
//!
//! The values are the ones the cog path actually runs:
//! [`MotionConfig::default`], [`TrackingFaultConfig::default`] and the phase
//! constants, never a copy of them in some host's configuration file. A
//! re-derived figure landing outside a stated tolerance is escalated for human
//! review as a suspected computation difference; the pin is never moved and no
//! tolerance widened to make this suite green.

mod replay_trace;

use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{ROWS, flags, group_of};
use reachy_motion::tick::{RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_HEAD_LAG_RAD, tracking};
use reachy_motion::{
    ANTENNA_PHASE_SEPARATION_RAD, JointGroup, JointRef, JointTargets, MotionCommand, MotionConfig,
    MotionSnapWire, MoveDurations, WarpKind, dry_pass_peaks, floor_move_clock, stow_pose_targets,
    stow_targets,
};

use replay_trace::{ARRIVED_TOLERANCE_RAD, Run, Sample, Trace, fixture};

/// The rate the recordings were driven at and the shipped floors are derived
/// at.
const TICK_HZ: f64 = reachy_motion::FLOOR_TICK_HZ;

/// The head clock every recorded gesture was commanded over — the shipped
/// `up_duration`.
const UP: Duration = Duration::from_millis(800);

/// Degrees as radians, for a figure measured in the units the machine is
/// measured in.
fn deg(degrees: f64) -> f64 {
    degrees.to_radians()
}

/// The pose a recorded run started from, as the command path expresses one.
///
/// The head is the stow posture, which is where the recordings begin to within
/// a degree — asserted, because the fixtures only speak for the shipped
/// posture while that holds. A degree and not a count because the servos were
/// holding stow on the gains of that night, and holding a weight up on a
/// proportional term alone parks a loaded crank a little short of where it was
/// sent. The antennas come off the recording: an antenna direction has a
/// representative per turn, and the sweep the resolver picks depends on which
/// one the machine stood at.
fn started_at(cfg: &MotionConfig, run: &Run) -> JointTargets {
    let present = run.samples[0].present.expect("the first period read");
    let held = stow_targets(&cfg.geom).expect("the geometry reaches stow");
    for (leg, angle) in present.legs.iter().enumerate() {
        assert!(
            (angle - held.legs[leg]).abs() < deg(1.0),
            "leg {leg} began at {angle}, not the stow pose's {}",
            held.legs[leg]
        );
    }
    JointTargets {
        antennas: present.antennas,
        ..stow_pose_targets()
    }
}

/// The gesture the recordings are of: stow to neutral, on `durations`.
fn gesture(durations: MoveDurations) -> MotionCommand {
    MotionCommand::MoveTo {
        target: JointTargets::default(),
        durations,
        warp: WarpKind::MinJerk,
    }
}

/// The shipped tracking comparison driven over a recorded run, period by
/// period, answering the joints whose window ran out and when.
///
/// A period whose grouped read fell short is skipped: a stale measurement
/// would freeze the run where it stands. A released joint is handed over
/// masked — it holds no goal to lag behind.
fn trips(cfg: &MotionConfig, run: &Run) -> Vec<(u64, JointFlags)> {
    let mut state = MotionSnapWire::new();
    let state = state.clear_valid();
    let mut out = Vec::new();
    for sample in &run.samples {
        let Some(present) = sample.present else {
            continue;
        };
        // A released joint is commanded nothing, so it stands at its own angle
        // rather than at a goal it never had.
        let mut goal = present;
        for joint in ROWS {
            if let Some(angle) = sample.goal_of(joint) {
                goal.set(joint, angle);
            }
        }
        let look = tracking::look(
            &cfg.tracking,
            sample.released(),
            &present,
            &goal,
            &mut state.tracking,
        );
        if !flags::is_empty(look.exhausted) {
            out.push((sample.tick, look.exhausted));
        }
    }
    out
}

/// Guard 1. Neither run that went well raises anything in the shipped tracking
/// comparison, and the lags they ran at are the headroom record.
///
/// The validated gesture and the fastest sweep on record, measured on the
/// machine, both with every joint following its goal at a distance the whole
/// way through — a quarter of a radian on a loaded leg, and better than a
/// radian on the antenna crossing 187° in four tenths of a second. A threshold
/// sized under those, or a progress minimum over what the machine closes in a
/// window, shows up here as a fault on a gesture the machine is known to make
/// well.
#[test]
fn the_runs_that_went_well_raise_nothing() {
    let cfg = MotionConfig::default();
    for name in ["trace-verify2", "trace-fast4"] {
        let trace = fixture(name);
        let trips = trips(&cfg, trace.run(0));
        assert!(
            trips.is_empty(),
            "{name}: {:?}",
            trips
                .iter()
                .map(|(at, out)| (*at, flags::Names(*out).to_string()))
                .collect::<Vec<_>>()
        );
    }

    // The two lags the default threshold is sized over, pinned where they were
    // measured: the loaded leg on the validated gesture, and the antenna on the
    // 855°/s sweep. Both sit under the threshold's own headroom claim. The
    // figures are the library's own constants, so the recordings and the number
    // a live run is reported against are one statement.
    let head_lag = ROWS
        .into_iter()
        .filter(|joint| group_of(*joint) != Some(JointGroup::Antennas))
        .map(|joint| fixture("trace-verify2").run(0).joint(joint).worst_lag)
        .fold(0.0_f64, f64::max);
    assert!(
        (head_lag - RECORDED_WORST_HEAD_LAG_RAD).abs() < 5e-3,
        "the validated gesture's worst head lag is {head_lag:.4} rad"
    );
    let fast4 = fixture("trace-fast4");
    let antenna_lag = [JointRef::AntennaRight, JointRef::AntennaLeft]
        .into_iter()
        .map(|joint| fast4.run(0).joint(joint).worst_lag)
        .fold(0.0_f64, f64::max);
    assert!(
        (antenna_lag - RECORDED_WORST_ANTENNA_LAG_RAD).abs() < 5e-3,
        "the fast sweep's worst antenna lag is {antenna_lag:.4} rad"
    );
    assert!(
        head_lag < cfg.tracking.threshold_rad,
        "the healthy head lag {head_lag:.4} rad now reaches the {:.4} rad threshold",
        cfg.tracking.threshold_rad
    );
}

/// Guard 1. The lag-and-speed pairs the shipped tracking comment quotes are
/// what the recordings hold.
///
/// Not a fault case: nothing here crosses a threshold. It is the documentation
/// guard behind `TrackingFaultConfig`'s argument that lag scales with commanded
/// speed — a leg and an antenna each pinned as a lag beside the speed of the
/// goal it was chasing, so a re-recording or a shaper change moves the comment
/// rather than leaving its figures standing as folklore. Both figures of a pair
/// come from one joint: a lag read against a speed some other joint was
/// commanded at would support nothing.
#[test]
fn the_lag_and_speed_figures_the_tracking_comment_quotes_are_what_the_recordings_hold() {
    let verify2 = fixture("trace-verify2");
    // Leg 2 as the bus numbers the servos, which is the second leg row.
    let leg = verify2.run(0).joint(JointRef::Leg1);
    assert!(
        (leg.worst_lag - 0.245).abs() < 5e-3,
        "leg 2's lag on the validated gesture is {:.4} rad",
        leg.worst_lag
    );
    assert!(
        (leg.peak_goal_speed - 3.34).abs() < 5e-3,
        "the goal leg 2 was following peaks at {:.4} rad/s",
        leg.peak_goal_speed
    );

    let worst_antenna = |trace: &Trace| {
        [JointRef::AntennaRight, JointRef::AntennaLeft]
            .into_iter()
            .map(|joint| trace.run(0).joint(joint))
            .max_by(|left, right| left.worst_lag.total_cmp(&right.worst_lag))
            .expect("the pair is not empty")
    };
    let antenna = worst_antenna(&verify2);
    assert!(
        (antenna.worst_lag - 0.82).abs() < 5e-3,
        "the antennas' lag on the validated gesture is {:.4} rad",
        antenna.worst_lag
    );
    assert!(
        (antenna.peak_goal_speed - 7.55).abs() < 5e-3,
        "the goal that antenna was following peaks at {:.4} rad/s",
        antenna.peak_goal_speed
    );

    // The fast sweep's own pair. The 855°/s the comment names is what the joint
    // reached, pinned by the speed-record case below; what it was asked for is
    // half again as fast, and that is the speed its 1.38 rad of lag is read
    // against.
    let fast_antenna = worst_antenna(&fixture("trace-fast4"));
    assert!(
        (fast_antenna.worst_lag - RECORDED_WORST_ANTENNA_LAG_RAD).abs() < 5e-3,
        "the fast sweep's worst antenna lag is {:.4} rad",
        fast_antenna.worst_lag
    );
    assert!(
        (fast_antenna.peak_goal_speed - deg(1123.0)).abs() < deg(2.0),
        "the goal that antenna was following peaks at {:.1} deg/s",
        fast_antenna.peak_goal_speed.to_degrees()
    );
}

/// Guard 2. The one collision on record trips it, on the pair that stalled and
/// on nothing else.
///
/// Both antenna tips met at the crossing and stood there for over forty periods
/// with the goal three radians away; the head carried on and arrived. So the
/// comparison has to name the antennas and only the antennas — which is what
/// decides the response: the pair goes out of service and the head move
/// finishes, rather than the whole machine winding down.
#[test]
fn the_collision_trips_it_on_the_antennas_and_nothing_else() {
    let cfg = MotionConfig::default();
    let trace = fixture("trace-stagger");
    let trips = trips(&cfg, trace.run(2));

    assert!(
        !trips.is_empty(),
        "the stalled pair never ran its window out"
    );
    for (at, exhausted) in &trips {
        for joint in flags::iter(*exhausted) {
            assert_eq!(
                group_of(joint),
                Some(JointGroup::Antennas),
                "a head joint ran its window out at period {at}: {}",
                flags::Names(*exhausted)
            );
        }
    }
    // Both sides, and inside one window of each other: they met each other, so
    // neither is the one that failed. Two antennas stalling hundreds of periods
    // apart would be two single-servo faults, which is a different condition
    // with a different answer.
    let ran_out = |side| {
        trips
            .iter()
            .find(|(_, out)| flags::contains(*out, side))
            .map(|(at, _)| *at)
    };
    let right = ran_out(JointRef::AntennaRight).expect("the right antenna stalled");
    let left = ran_out(JointRef::AntennaLeft).expect("the left antenna stalled");
    assert!(
        right.abs_diff(left) <= u64::from(cfg.tracking.ticks),
        "the antennas ran their windows out at periods {right} and {left}, further apart than \
         the {} the window itself is: one stalled and the other did not",
        cfg.tracking.ticks
    );
}

/// Guard 3. The step bounds admit the gestures that were recorded, with the
/// headroom the shipped figures claim.
///
/// Against the *plan* and never against the record. The recorded goal column is
/// what the loop commanded, and these recordings predate the per-period move
/// clock, so a period that started late sampled the trajectory further along
/// and commanded a step no planner ever asked for. The last case below is that
/// inflation, pinned: the fastest sweep's recorded step is past the bound its
/// own plan clears comfortably.
#[test]
fn the_step_bounds_admit_the_recorded_gestures_with_headroom() {
    let cfg = MotionConfig::default();
    let verify2 = fixture("trace-verify2");
    let fast4 = fixture("trace-fast4");

    // The validated gesture, on the clock the cog path ships, and the same
    // gesture with the staggered antenna pair the recording was made on —
    // whose quick side is the 0.3 s sweep the speed record was set on.
    let cases = [
        (
            "the validated gesture",
            started_at(&cfg, verify2.run(0)),
            gesture(MoveDurations::uniform(UP)),
        ),
        (
            "the staggered pair",
            started_at(&cfg, fast4.run(0)),
            gesture(MoveDurations {
                head: UP,
                antennas: [Duration::from_millis(700), Duration::from_millis(300)],
            }),
        ),
    ];

    let mut planned = Vec::new();
    for (name, start, command) in cases {
        let peaks =
            dry_pass_peaks(&cfg, &start, &command, TICK_HZ).expect("the gesture is measurable");
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
                "{name}: the {side} antenna plans {peak:.4} rad against the {:.4} rad bound",
                cfg.max_step.antennas
            );
        }
        // And no clock needs right-sizing for its span, which is the same
        // statement the shipped durations make about their floors. The pass may
        // still lengthen an antenna to de-phase the pair — the head's clock is
        // what says nothing was floored.
        let stretch = floor_move_clock(&cfg, &start, &command, TICK_HZ).1;
        assert!(
            stretch.is_none_or(
                |clocks| clocks.dephased && clocks.effective.head == clocks.requested.head
            ),
            "{name}: the shipped clock does not carry it"
        );
    }

    // The inflation, pinned. On the quick side the loop commanded half as much
    // again in a period as the planner ever asked for, because the periods it
    // woke on were half as long again as the grid it was sampling. A bound
    // sized to clear the record by the same margin would be half as wide again
    // for no reason the plan gives.
    let quick = planned[1].antennas[1];
    let recorded = fast4.run(0).joint(JointRef::AntennaLeft).peak_goal_step;
    assert!(
        recorded > quick * 1.4,
        "the recorded step {recorded:.4} rad is no longer inflated over the planned {quick:.4} \
         rad, so this case no longer says what it is for"
    );
}

/// Guard 4. The separation a pair is held to admits the clock pair that swept
/// clean and rejects the one that clashed.
///
/// Both figures come off the recordings' own commanded goals, which is where a
/// clock pair's phase is visible. The pair that clashed is the binding end: the
/// offset it stood at when the second tip reached the band is well under the
/// constant, so nothing phased like that gets through. The stow that clashed
/// and the raise that did not are the same two clocks recorded twice — the
/// raise got through on the odds the debrief measured, about two inboard sweeps
/// in three, and the check rejects it too.
#[test]
fn the_separation_tells_the_clean_pair_from_the_one_that_clashed() {
    let cfg = MotionConfig::default();
    let band = cfg.phase.contact_band_rad;
    let stagger = fixture("trace-stagger");
    let fast4 = fixture("trace-fast4");

    let clean = fast4
        .run(0)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        clean.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the validated pair plans {:.4} rad, under the shipped separation",
        clean.offset
    );
    assert!(
        (clean.offset - 0.876).abs() < 5e-3,
        "the validated pair plans {:.4} rad",
        clean.offset
    );

    let clashed = stagger
        .run(2)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        !clashed.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the pair that clashed planned {:.4} rad, which the shipped separation now admits",
        clashed.offset
    );
    assert!(
        (clashed.offset - 0.361).abs() < 5e-3,
        "the pair that clashed planned {:.4} rad",
        clashed.offset
    );

    let survived = stagger
        .run(1)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        !survived.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the raise on the clashing clocks planned {:.4} rad, which the shipped separation now \
         admits",
        survived.offset
    );

    // And what the tips themselves did, which is what the plan is a proxy for:
    // on the raise they passed the band's edge a third of a radian apart, and on
    // the stow they never reached it at all — they met inside the band and
    // stalled there.
    let tips = stagger
        .run(1)
        .separation(band, Sample::present_of)
        .expect("both antennas cross the band");
    assert!(
        (tips.offset - 0.285).abs() < 5e-3,
        "the tips passed {:.4} rad apart",
        tips.offset
    );
    assert!(
        stagger
            .run(2)
            .separation(band, Sample::present_of)
            .is_none(),
        "the jammed pair left the band after all"
    );
}

/// Guard 5, one test per fixture. The validated gesture: the whole machine,
/// head and both antennas, up in 0.82 s and measurably there before the last
/// goal went out.
///
/// Zero settle is the claim this recording is kept for — not one period was
/// spent waiting after the commanding stopped — and it is the shape every guard
/// in this crate has to admit.
#[test]
fn the_validated_gesture_arrives_inside_its_own_clock() {
    let trace = fixture("trace-verify2");
    assert_eq!(trace.runs(), 1);
    let run = trace.run(0);

    assert_eq!(run.samples.len(), 27);
    assert!(
        (run.span().as_secs_f64() - 0.8196).abs() < 5e-4,
        "{:?}",
        run.span()
    );
    assert_eq!(run.commanding_end(), Some(run.span()));
    for joint in ROWS {
        let metrics = run.joint(joint);
        let arrived = metrics.arrived.expect("it got there");
        assert!(
            arrived <= run.span(),
            "{joint:?} arrived at {arrived:?}, after the commanding stopped"
        );
        let residual = metrics.residual.expect("it was holding a goal");
        assert!(residual < ARRIVED_TOLERANCE_RAD, "{joint:?}: {residual}");
    }
    // The fastest leg was asked for 0.107 rad in a period. That is more than
    // the same command's dry-pass step: this run was driven by a loop that
    // sampled the trajectory at wall-clock time, so a period that started late
    // commanded the extra. It is recorded, and nothing is sized against it.
    let leg = run.joint(JointRef::Leg1);
    assert!(
        (leg.peak_goal_step - 0.1068).abs() < 5e-4,
        "{}",
        leg.peak_goal_step
    );
    assert!(
        (leg.peak_speed - deg(199.3)).abs() < deg(0.5),
        "{}",
        leg.peak_speed
    );
}

/// Guard 5. The antenna speed record, and the staggered pair that made it safe:
/// one side sweeping 187° in 0.40 s at 855°/s while the other takes 0.93 s over
/// the same arc.
#[test]
fn the_fast_sweep_is_the_speed_record_and_a_staggered_pair() {
    let run = fixture("trace-fast4");
    let run = run.run(0);
    let fast = run.joint(JointRef::AntennaLeft);
    let slow = run.joint(JointRef::AntennaRight);

    assert!((fast.span - deg(187.0)).abs() < deg(0.5), "{}", fast.span);
    assert!(
        (fast.peak_speed - deg(855.5)).abs() < deg(1.0),
        "{}",
        fast.peak_speed
    );
    let arrived = fast.arrived.expect("it got there");
    assert!((arrived.as_secs_f64() - 0.4035).abs() < 5e-4, "{arrived:?}");
    // The other side is on its own clock, more than half a second behind —
    // which is the whole point of the pair having two.
    let behind = slow.arrived.expect("it got there") - arrived;
    assert!(behind > Duration::from_millis(500), "{behind:?}");
    assert!(
        slow.peak_speed < fast.peak_speed / 2.0,
        "{} against {}",
        slow.peak_speed,
        fast.peak_speed
    );
}

/// Guard 5. The gain change, recorded as the same step command twice: the
/// P-only gains of that night park the loaded pair ~4° short of the goal for
/// good, and the tuned gains bring that to about a degree.
///
/// Neither run is a clean any bound can be replayed against. Both command the
/// whole span in one period by construction — that is what a step response is —
/// so the goal steps here are records of the instrument, not of a move anything
/// should admit.
#[test]
fn the_gain_change_is_two_step_responses_and_the_droop_between_them() {
    let trace = fixture("trace-newgains");
    assert_eq!(trace.runs(), 2);
    let shipped = trace.run(0);
    let tuned = trace.run(1);

    for (index, run) in [(0, shipped), (1, tuned)] {
        for joint in ROWS {
            let metrics = run.joint(joint);
            if metrics.span > deg(10.0) {
                assert!(
                    metrics.peak_goal_step > metrics.span / 2.0,
                    "run {index}, {joint:?}: the goal jumped {} of a {} span",
                    metrics.peak_goal_step,
                    metrics.span
                );
            }
        }
    }

    // Legs 2 and 5 carry the load. Under the gains of that night they stop
    // short and stay short — the steady-state droop of a proportional term
    // holding gravity.
    for leg in [JointRef::Leg1, JointRef::Leg4] {
        let droop = shipped.joint(leg);
        let residual = droop.residual.expect("it was holding a goal");
        assert!(
            (deg(3.9)..=deg(4.4)).contains(&residual),
            "{leg:?}: {}",
            residual.to_degrees()
        );
        assert_eq!(droop.arrived, None, "{leg:?} never got there");

        let after = tuned.joint(leg);
        let residual = after.residual.expect("it was holding a goal");
        assert!(residual < deg(1.3), "{leg:?}: {}", residual.to_degrees());
        assert!(after.arrived.is_some(), "{leg:?} got there");
    }
}

/// Guard 5. The one collision on record: both antenna tips meet at the inboard
/// crossing, stall against each other at mirrored angles for over 40 periods —
/// about 1.06 s — while the goal walks away, and spring back when the servos
/// give up.
///
/// This is the run every guard has to catch. It is the third in its file — the
/// same session's earlier raise, run 1, went through cleanly.
#[test]
fn the_collision_stalls_both_antennas_at_mirrored_angles() {
    let trace = fixture("trace-stagger");
    assert_eq!(trace.runs(), 3);
    let jam = trace.run(2);
    let right = jam
        .longest_stall(JointRef::AntennaRight, deg(1.0))
        .expect("it stopped");
    let left = jam
        .longest_stall(JointRef::AntennaLeft, deg(1.0))
        .expect("it stopped");

    // Mirrored, which is what tip-to-tip means: the two sides stop at the same
    // angle on opposite sides, a few degrees apart.
    for (side, stall) in [("right", right), ("left", left)] {
        assert!(
            (deg(52.0)..=deg(56.6)).contains(&stall.at.abs()),
            "{side}: {}",
            stall.at.to_degrees()
        );
        // Held there while the goal ran the rest of the way home.
        assert!(stall.periods >= 40, "{side}: {} periods", stall.periods);
        assert!(
            stall.worst_lag > deg(120.0),
            "{side}: {}",
            stall.worst_lag.to_degrees()
        );
    }
    assert!(
        right.at.signum() != left.at.signum(),
        "opposite sides: {} and {}",
        right.at.to_degrees(),
        left.at.to_degrees()
    );
    assert!(
        (right.at + left.at).abs() < deg(5.0),
        "{} against {}",
        right.at.to_degrees(),
        left.at.to_degrees()
    );

    // Neither arrived, and both finished further from the goal than they
    // stalled: the tips sprang apart as the servos dropped out.
    for (joint, stall) in [
        (JointRef::AntennaRight, right),
        (JointRef::AntennaLeft, left),
    ] {
        let antenna = jam.joint(joint);
        assert_eq!(antenna.arrived, None, "{joint:?}");
        let residual = antenna.residual.expect("it was holding a goal");
        assert!(residual > stall.worst_lag, "{joint:?}: {residual}");
    }
}
