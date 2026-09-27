//! A caller-supplied motion script and its checked sender and launcher clocks.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

use motion_proto::MotionScript;

use crate::tour::{
    BUDGET_MARGIN_S, COMMISSIONING_ALLOWANCE_MS, END_MARGIN_MS, RELEASE_ALLOWANCE_MS,
};

#[derive(Clone, Debug, PartialEq)]
pub struct ScriptPlan {
    path: PathBuf,
    body: String,
    script: MotionScript,
}

impl ScriptPlan {
    pub fn read(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref().to_owned();
        let body = fs::read_to_string(&path)
            .map_err(|error| format!("reading script {}: {error}", path.display()))?;
        let script = MotionScript::decode(&body)
            .map_err(|error| format!("decoding script {}: {error}", path.display()))?;
        Ok(Self { path, body, script })
    }

    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }
    #[must_use]
    pub fn body(&self) -> &str {
        &self.body
    }
    #[must_use]
    pub fn timeout_ms(&self) -> u64 {
        self.script.timeout_ms()
    }

    pub fn sender_deadline(&self) -> Result<Duration, String> {
        sender_deadline(self.timeout_ms())
    }

    pub fn launcher_budget(&self) -> Result<u64, String> {
        launcher_budget(self.timeout_ms())
    }
}

fn sender_deadline(timeout_ms: u64) -> Result<Duration, String> {
    let ms = timeout_ms
        .checked_add(RELEASE_ALLOWANCE_MS)
        .and_then(|value| value.checked_add(END_MARGIN_MS))
        .ok_or_else(|| "script sender deadline overflows milliseconds".to_owned())?;
    Ok(Duration::from_millis(ms))
}

fn launcher_budget(timeout_ms: u64) -> Result<u64, String> {
    let ms = COMMISSIONING_ALLOWANCE_MS
        .checked_add(timeout_ms)
        .and_then(|value| value.checked_add(RELEASE_ALLOWANCE_MS))
        .and_then(|value| value.checked_add(END_MARGIN_MS))
        .ok_or_else(|| "script launcher budget overflows milliseconds".to_owned())?;
    let seconds = ms
        .checked_add(999)
        .ok_or_else(|| "script launcher budget overflows rounding".to_owned())?
        / 1000;
    seconds
        .checked_add(BUDGET_MARGIN_S)
        .ok_or_else(|| "script launcher budget overflows seconds".to_owned())
}

#[cfg(test)]
mod tests {
    use super::ScriptPlan;
    use clockwork_rs::SyncTime;
    use motion_proto::{Action, MotionScript, STOW_POSE, Step};
    use reachy_edge::{EdgeConfig, HostEdge, Origin, Surface};
    use reachy_scratch::scratch_dir;
    use std::fs;
    use std::time::Duration;

    const GREETING: &str = include_str!("../fixtures/greeting.json");
    const SETTLE_EVIDENCE: &[(&str, &str)] = &[
        (
            "settle-evidence-1",
            include_str!("../fixtures/settle-evidence-1.json"),
        ),
        (
            "settle-evidence-2",
            include_str!("../fixtures/settle-evidence-2.json"),
        ),
    ];
    const LOOK_SWEEPS: &[(&str, &str)] = &[
        (
            "look-sweep-1",
            include_str!("../fixtures/look-sweep-1.json"),
        ),
        (
            "look-sweep-2",
            include_str!("../fixtures/look-sweep-2.json"),
        ),
    ];

    /// How long every settle script holds a target once its move's clock has
    /// run out: the stillness watch's own settle allowance.
    const HOLD_MS: u64 = 4_000;

    /// The bearings the look sweeps face, degrees.
    const SWEEP_BEARINGS_DEG: [f64; 13] = [
        0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0,
    ];

    /// The talker elevation every sweep look carries, degrees.
    const SWEEP_ELEVATION_DEG: f64 = 27.0;

    /// The head share the look sweeps' overlay run is flown under, degrees;
    /// the other run is flown at the shipped `scenario::LOOK_HEAD_SHARE_RAD`.
    const SWEEP_OVERLAY_SHARE_DEG: f64 = 45.0;

    struct Sink {
        lines: Vec<String>,
    }

    impl Surface for Sink {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }
        fn alert(&mut self, _alert: &reachy_edge::Alert) {}
    }

    fn plan(timeout_ms: u64) -> ScriptPlan {
        let dir = scratch_dir("reachy-ask-script");
        let path = dir.join("script.json");
        let body = MotionScript::new("reachy-ask", 1, vec![Step::new(1, STOW_POSE)], timeout_ms)
            .expect("test script")
            .encode();
        fs::write(&path, body).expect("script");
        ScriptPlan::read(path).expect("decoded plan")
    }

    #[test]
    fn arithmetic_uses_the_shared_allowances() {
        let plan = plan(33_500);
        assert_eq!(plan.timeout_ms(), 33_500);
        assert_eq!(
            plan.sender_deadline().expect("deadline"),
            Duration::from_millis(42_500)
        );
        assert_eq!(plan.launcher_budget().expect("budget"), 79);
    }

    #[test]
    fn malformed_and_unreadable_files_are_refused() {
        let dir = scratch_dir("reachy-ask-bad");
        let bad = dir.join("bad.json");
        let missing = dir.join("missing.json");
        fs::write(&bad, "not json").expect("bad file");
        assert!(ScriptPlan::read(bad).is_err());
        assert!(ScriptPlan::read(missing).is_err());
    }

    #[test]
    fn checked_timing_rejects_overflow() {
        assert!(super::sender_deadline(u64::MAX).is_err());
        assert!(super::launcher_budget(u64::MAX).is_err());
    }

    #[test]
    fn the_committed_greeting_decodes_with_the_embedded_edge() {
        let dir = scratch_dir("reachy-ask-greeting");
        let path = dir.join("greeting.json");
        fs::write(&path, GREETING).expect("greeting fixture");
        let plan = ScriptPlan::read(path).expect("decoded greeting");
        assert_eq!(plan.timeout_ms(), 33_500);
        assert_eq!(plan.launcher_budget().expect("launcher budget"), 79);
        let steps = &plan.script.steps();
        assert_eq!(steps.len(), 6);
        assert_eq!(steps[0].after_ms, 8_000);
        assert_eq!(
            steps[0].action.base().and_then(motion_proto::Base::pose),
            Some("peek_tilt")
        );
        assert_eq!(steps[1].after_ms, 10_000);
        assert_eq!(
            steps[1].action.base().and_then(motion_proto::Base::pose),
            Some("hello")
        );
        assert_eq!(steps[2].after_ms, 11_200);
        assert_eq!(
            steps[2]
                .action
                .play()
                .map(|play| (play.name.as_str(), play.speed)),
            Some(("hello_wave", 1.0))
        );
        assert_eq!(steps[3].after_ms, 16_000);
        assert_eq!(
            steps[3].action.base().and_then(motion_proto::Base::pose),
            Some("neutral")
        );
        assert_eq!(steps[4].after_ms, 17_200);
        assert_eq!(
            steps[4]
                .action
                .play()
                .map(|play| (play.name.as_str(), play.speed)),
            Some(("dance", 1.0))
        );
        assert_eq!(steps[5].after_ms, 31_500);
        assert_eq!(
            steps[5].action.base().and_then(motion_proto::Base::pose),
            Some("stow")
        );
        assert!(
            steps
                .iter()
                .all(|step| !matches!(step.action, Action::Base(motion_proto::Base::Keep)))
        );
        let stow = crate::gesture::poses()
            .resolve(motion_proto::STOW_POSE)
            .expect("stow is in the committed pose library");
        assert_eq!(
            plan.timeout_ms() - steps[5].after_ms,
            u64::from(stow.duration_ms)
        );
        for step in &steps[..5] {
            if let Some(play) = step.action.play() {
                let entry = crate::gesture::motions()
                    .resolve(&play.name)
                    .unwrap_or_else(|| panic!("{} is in the committed motion library", play.name));
                let end = step.after_ms + entry.window.span_ms(play.speed);
                assert!(
                    end <= steps[5].after_ms,
                    "{} runs from {} to {}, past the stow at {}",
                    play.name,
                    step.after_ms,
                    end,
                    steps[5].after_ms
                );
            }
        }

        let mut edge = HostEdge::new(
            EdgeConfig::for_pod(crate::gesture::ASK_POD),
            crate::gesture::motions().clone(),
            crate::gesture::poses().clone(),
        );
        let mut sink = Sink { lines: Vec::new() };
        let accepted = edge
            .offer(
                GREETING.as_bytes(),
                Origin::Local,
                SyncTime::from_nanos(1_700_000_000_000_000_000),
                &mut sink,
            )
            .unwrap_or_else(|| panic!("greeting refused:\n{}", sink.lines.join("\n")));
        assert_eq!(accepted.seq, 1);
        assert_eq!(accepted.script_id, 1);
        let plays: Vec<_> = steps
            .iter()
            .filter_map(|step| step.action.play().map(|play| (step.after_ms, play)))
            .collect();
        let overlays = accepted.message.overlays();
        assert_eq!(overlays.len(), plays.len());
        for ((after_ms, play), overlay) in plays.iter().zip(overlays.iter()) {
            let entry = crate::gesture::motions()
                .resolve(&play.name)
                .expect("the accepted play resolves in the committed motion library");
            assert_eq!(overlay.motion_id(), entry.motion_id, "{} motion", play.name);
            assert_eq!(overlay.after_ms(), u32::try_from(*after_ms).unwrap());
            assert_eq!(
                overlay.duration_ms(),
                u32::try_from(entry.window.span_ms(play.speed)).unwrap(),
                "{} duration",
                play.name
            );
            assert_eq!(overlay.speed(), play.speed, "{} speed", play.name);
        }
    }

    /// `degrees` to the nearest milliradian, the wire's look unit.
    #[expect(
        clippy::cast_possible_truncation,
        reason = "every angle here is at most 120 degrees, about 2094 milliradians"
    )]
    fn mrad(degrees: f64) -> i32 {
        (degrees.to_radians() * 1000.0).round() as i32
    }

    fn hold_is_the_settle_allowance() {
        assert_eq!(
            u128::from(HOLD_MS),
            reachy_motion::stillness::StillnessConfig::default()
                .settle
                .as_millis()
        );
    }

    /// The clock the Mover floors a move from `from` to `to` on `pace` to, at
    /// the shipped configuration, in whole milliseconds rounded up: the longest
    /// of the head's and the two antennas' clocks.
    fn clock_ms(
        from: &reachy_motion::joints::JointTargets,
        to: &reachy_motion::joints::JointTargets,
        pace: Duration,
    ) -> u64 {
        use reachy_motion::tick::{MotionCommand, default_motion_config, floor_move_clock};
        use reachy_motion::traj::{MoveDurations, WarpKind};
        let command = MotionCommand::MoveTo {
            target: *to,
            durations: MoveDurations::uniform(pace),
            warp: WarpKind::MinJerk,
        };
        #[expect(
            clippy::cast_precision_loss,
            reason = "the period is 20 ms in nanoseconds, exact in an f64"
        )]
        let tick_hz = 1e9 / scenario::PERIOD_NS as f64;
        let (floored, _) = floor_move_clock(default_motion_config(), from, &command, tick_hz);
        let MotionCommand::MoveTo { durations, .. } = floored else {
            panic!("a floored move is a move")
        };
        let longest = durations
            .head
            .max(durations.antennas[0])
            .max(durations.antennas[1]);
        u64::try_from(longest.as_nanos().div_ceil(1_000_000)).expect("a clock in ms")
    }

    /// Where `step` sends the machine and the pace it asks: a pose's library
    /// targets and duration, or a look's composition at `head_share` on the
    /// deployed look clock.
    fn targets_of(step: &Step, head_share: f64) -> (reachy_motion::joints::JointTargets, Duration) {
        match step.action.base() {
            Some(motion_proto::Base::Pose { name, .. }) => scenario::pose_library()
                .targets(scenario::pose_id(name))
                .unwrap_or_else(|| panic!("{name} is in the committed pose library")),
            Some(motion_proto::Base::Look {
                bearing_mrad,
                elevation_mrad,
            }) => (
                scenario::look_pose_at(*bearing_mrad, *elevation_mrad, head_share),
                Duration::from_millis(
                    u64::try_from(scenario::LOOK_HEAD_MS).expect("a positive look clock"),
                ),
            ),
            other => panic!("a settle script steps to a pose or a look, not {other:?}"),
        }
    }

    /// The offsets and timeout a settle script's steps must carry: the first
    /// move at 8 s, each next one when the previous move's floored clock has
    /// run and the target has been held [`HOLD_MS`], and the timeout when the
    /// closing stow's clock has run. A move's clock is the longer of its
    /// floored clocks at the two shares the scripts are flown under, the
    /// shipped one and [`SWEEP_OVERLAY_SHARE_DEG`], so every hold is at least
    /// [`HOLD_MS`] at both. Each move starts from the previous step's target at
    /// the same share, the first from the stow.
    fn expected_timing(steps: &[Step]) -> (Vec<u64>, u64) {
        let shares = [
            scenario::LOOK_HEAD_SHARE_RAD,
            SWEEP_OVERLAY_SHARE_DEG.to_radians(),
        ];
        let mut offsets = Vec::with_capacity(steps.len());
        let mut from = [scenario::stow_pose(); 2];
        let mut at = 8_000;
        let mut last_clock = 0;
        for step in steps {
            if !offsets.is_empty() {
                at += last_clock + HOLD_MS;
            }
            offsets.push(at);
            last_clock = 0;
            for (start, share) in from.iter_mut().zip(shares) {
                let (to, pace) = targets_of(step, share);
                last_clock = last_clock.max(clock_ms(start, &to, pace));
                *start = to;
            }
        }
        (offsets, at + last_clock)
    }

    /// The script's own offsets and timeout against [`expected_timing`]'s,
    /// printing both whole on a mismatch.
    fn assert_timing(label: &str, script: &MotionScript) {
        let steps = script.steps();
        let actual: Vec<u64> = steps.iter().map(|step| step.after_ms).collect();
        let (expected, timeout) = expected_timing(steps);
        assert!(
            (actual.as_slice(), script.timeout_ms()) == (expected.as_slice(), timeout),
            "{label}: offsets {actual:?}, timeout {}; the rule gives offsets {expected:?}, \
             timeout {timeout}",
            script.timeout_ms()
        );
    }

    /// The script `body` as the embedded edge accepts it.
    fn accept(label: &str, body: &str) -> reachy_edge::Accepted {
        let mut edge = HostEdge::new(
            EdgeConfig::for_pod(crate::gesture::ASK_POD),
            crate::gesture::motions().clone(),
            crate::gesture::poses().clone(),
        );
        let mut sink = Sink { lines: Vec::new() };
        edge.offer(
            body.as_bytes(),
            Origin::Local,
            SyncTime::from_nanos(1_700_000_000_000_000_000),
            &mut sink,
        )
        .unwrap_or_else(|| panic!("{label} refused: {}", sink.lines.join("\n")))
    }

    /// The two walks hold every committed pose, and step between every ordered
    /// pair of them, on the timing rule: each move on the clock the Mover floors
    /// it to, then a 4 s hold.
    #[test]
    fn settle_evidence_walks_are_derived_from_the_committed_pose_table() {
        hold_is_the_settle_allowance();
        let poses = crate::gesture::poses();
        let by_name: std::collections::BTreeMap<_, _> = poses
            .entries()
            .map(|(name, entry)| (name, *entry))
            .collect();
        let mut pairs = std::collections::BTreeSet::new();
        let mut candidate_counts = Vec::new();

        for (label, body) in SETTLE_EVIDENCE {
            let script = MotionScript::decode(body).expect("settle fixture decodes");
            let steps = script.steps();
            assert!(!steps.is_empty());
            assert_eq!(steps[0].after_ms, 8_000);
            assert!(
                steps.iter().all(|step| step
                    .action
                    .base()
                    .and_then(motion_proto::Base::pose)
                    .is_some()),
                "{label}: every step is a pose"
            );
            assert_eq!(
                steps
                    .last()
                    .and_then(|step| step.action.base().and_then(motion_proto::Base::pose)),
                Some(motion_proto::STOW_POSE)
            );
            assert_timing(label, &script);
            assert!(steps.len() <= reachy_edge::compile::MAX_STEPS);

            let accepted = accept(label, body);
            assert_eq!(accepted.message.steps().len(), steps.len());
            assert!(accepted.message.overlays().is_empty());
            for (expected, actual) in steps.iter().zip(accepted.message.steps().iter()) {
                assert_eq!(
                    actual.pose_id(),
                    by_name
                        .get(
                            expected
                                .action
                                .base()
                                .and_then(motion_proto::Base::pose)
                                .expect("pose")
                        )
                        .expect("pose")
                        .pose_id
                );
                assert_eq!(
                    actual.after_ms(),
                    u32::try_from(expected.after_ms).expect("offset")
                );
            }
            let mut previous = motion_proto::STOW_POSE.to_owned();
            for step in &steps[..steps.len() - 1] {
                let current = step
                    .action
                    .base()
                    .and_then(motion_proto::Base::pose)
                    .expect("pose")
                    .to_owned();
                pairs.insert((previous.clone(), current.clone()));
                previous = current;
            }
            candidate_counts.push(steps.len() - 1);
        }

        assert_eq!(candidate_counts, vec![10, 12]);
        let names: Vec<_> = poses.entries().map(|(name, _)| name.to_owned()).collect();
        let expected: std::collections::BTreeSet<_> = names
            .iter()
            .flat_map(|from| {
                names
                    .iter()
                    .filter(move |to| from != *to)
                    .map(move |to| (from.clone(), (*to).clone()))
            })
            .collect();
        assert_eq!(pairs, expected);
    }

    /// The two look sweeps face every bearing from 0° to ±120° at 27°, start
    /// looks from `neutral`, `hello`, `peek_tilt` and from other looks, and
    /// hold each target at least 4 s after its move.
    ///
    /// The sweeps are flown at the shipped share and at the 45° overlay share.
    /// Each offset is set by whichever of the two shares floors the previous
    /// move longer, so every hold is at least 4 s at both shares and exactly
    /// 4 s at the share that set it.
    #[test]
    fn the_look_sweeps_face_every_bearing_from_every_start_and_hold_each() {
        use brenn_reachy__cogs__schedule_clk_rs::StepKindWire;
        hold_is_the_settle_allowance();
        let elevation = mrad(SWEEP_ELEVATION_DEG);
        let mut bearings_expected: Vec<i32> = SWEEP_BEARINGS_DEG.iter().map(|d| mrad(*d)).collect();
        bearings_expected.sort_unstable();
        let mut before_a_look = std::collections::BTreeSet::new();

        for (label, body) in LOOK_SWEEPS {
            let script = MotionScript::decode(body).expect("look sweep decodes");
            let steps = script.steps();
            assert_eq!(steps[0].after_ms, 8_000, "{label}");
            assert!(
                steps.iter().all(|step| matches!(
                    step.action.base(),
                    Some(motion_proto::Base::Pose { .. } | motion_proto::Base::Look { .. })
                )),
                "{label}: every step is a base step and none is a keep"
            );
            assert_eq!(
                steps
                    .last()
                    .and_then(|step| step.action.base().and_then(motion_proto::Base::pose)),
                Some(motion_proto::STOW_POSE),
                "{label}"
            );
            assert!(steps.len() <= reachy_edge::compile::MAX_STEPS, "{label}");

            let looks: Vec<(i32, i32)> = steps
                .iter()
                .filter_map(|step| match step.action.base() {
                    Some(motion_proto::Base::Look {
                        bearing_mrad,
                        elevation_mrad,
                    }) => Some((*bearing_mrad, *elevation_mrad)),
                    _ => None,
                })
                .collect();
            let mut bearings: Vec<i32> = looks.iter().map(|&(bearing, _)| bearing).collect();
            bearings.sort_unstable();
            assert_eq!(bearings, bearings_expected, "{label}");
            assert!(
                looks.iter().all(|&(_, e)| e == elevation),
                "{label}: {looks:?}"
            );
            assert_timing(label, &script);

            let mut look_after_look = false;
            for pair in steps.windows(2) {
                if let Some(motion_proto::Base::Look { .. }) = pair[1].action.base() {
                    match pair[0].action.base() {
                        Some(motion_proto::Base::Look { .. }) => look_after_look = true,
                        Some(motion_proto::Base::Pose { name, .. }) => {
                            before_a_look.insert(name.clone());
                        }
                        _ => {}
                    }
                }
            }
            assert!(look_after_look, "{label}: a look starts from a look");

            let accepted = accept(label, body);
            assert_eq!(accepted.message.steps().len(), steps.len(), "{label}");
            assert!(accepted.message.overlays().is_empty(), "{label}");
            for (expected, actual) in steps.iter().zip(accepted.message.steps().iter()) {
                match expected.action.base() {
                    Some(motion_proto::Base::Look {
                        bearing_mrad,
                        elevation_mrad,
                    }) => {
                        assert_eq!(actual.kind(), StepKindWire::BASE_LOOK, "{label}");
                        assert_eq!(actual.bearing_mrad(), *bearing_mrad, "{label}");
                        assert_eq!(actual.elevation_mrad(), *elevation_mrad, "{label}");
                    }
                    Some(motion_proto::Base::Pose { name, .. }) => {
                        assert_eq!(actual.pose_id(), scenario::pose_id(name), "{label}: {name}");
                    }
                    other => panic!("{label}: {other:?}"),
                }
                assert_eq!(
                    actual.after_ms(),
                    u32::try_from(expected.after_ms).expect("offset"),
                    "{label}"
                );
            }
        }

        for start in ["neutral", "hello", "peek_tilt"] {
            assert!(before_a_look.contains(start), "{start}: {before_a_look:?}");
        }
    }
}
