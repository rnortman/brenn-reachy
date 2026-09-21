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

    #[test]
    fn settle_evidence_walks_are_derived_from_the_committed_pose_table() {
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
            assert!(steps.iter().all(|step| step.action.base().is_some()));
            assert!(
                steps
                    .iter()
                    .all(|step| { !matches!(step.action, Action::Base(motion_proto::Base::Keep)) })
            );
            assert_eq!(
                steps
                    .last()
                    .and_then(|step| step.action.base().and_then(motion_proto::Base::pose)),
                Some(motion_proto::STOW_POSE)
            );
            for pair in steps.windows(2) {
                let from = pair[0]
                    .action
                    .base()
                    .and_then(motion_proto::Base::pose)
                    .expect("base pose");
                let duration = by_name.get(from).expect("known source pose").duration_ms;
                assert_eq!(
                    pair[1].after_ms,
                    pair[0].after_ms + u64::from(duration) + 1_500
                );
            }
            let last = steps.last().expect("last step");
            let stow = by_name.get(motion_proto::STOW_POSE).expect("stow");
            assert_eq!(
                script.timeout_ms(),
                last.after_ms + u64::from(stow.duration_ms)
            );
            assert!(steps.len() <= reachy_edge::compile::MAX_STEPS);

            let mut edge = HostEdge::new(
                EdgeConfig::for_pod(crate::gesture::ASK_POD),
                crate::gesture::motions().clone(),
                poses.clone(),
            );
            let mut sink = Sink { lines: Vec::new() };
            let accepted = edge
                .offer(
                    body.as_bytes(),
                    Origin::Local,
                    SyncTime::from_nanos(1_700_000_000_000_000_000),
                    &mut sink,
                )
                .unwrap_or_else(|| panic!("{label} refused: {}", sink.lines.join("\n")));
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
}
