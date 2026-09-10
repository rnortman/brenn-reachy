//! The probe documents, as the constants they step between.
//!
//! A probe is an instrument, and what makes it one is the shape of its frame
//! track: a pose reached in one frame and held, or a ramp of a stated length
//! between two poses. Both are stated here as a table — poses by name, segments
//! by frame count — and the JSON document under `cogs/clips/probe/` is written
//! from it by [`Probe::document`], which `gen_clip_config` calls before it walks
//! the directory.
//!
//! The poses are differences of the tree's own constants ([`NEUTRAL_ANTENNAS`],
//! [`ANTENNA_OUTBOARD`], [`STOW_ANTENNAS`]), never literals: a document
//! authored against a fold the tree has since moved loads, emits and holds
//! exactly as happily, and reads as the instrument it no longer is. Writing the
//! frames from the constants is what keeps that from being possible.
//!
//! Two frame conventions, and the difference between them is the whole
//! distinction between the two kinds of probe:
//!
//! - A [`Segment::Hold`] is `frames` copies of one pose. A hold that follows a
//!   hold of another pose is a **step**: the servo's own profile generator is
//!   the whole of the move and the arrival is a hard stop, which is what the
//!   stillness watch is pointed at.
//! - A [`Segment::Ramp`] emits `frames` frames from `from` towards `to`,
//!   *excluding* `to` — the segment after it supplies that. So a ramp followed
//!   by a hold arrives exactly once, and two ramps back to back reverse on a
//!   single frame with no dwell between them. The chain is checked rather than
//!   trusted: a ramp whose successor does not start where it was heading never
//!   reaches its pose, which no reader of the table would see.

use std::fmt::Write as _;

use anyhow::bail;

use reachy_clips::format::{CLIP_KIND, Channel, ClipDoc, FORMAT_VERSION, FrameDoc};
use reachy_motion::ANTENNA_OUTBOARD;
use reachy_motion::disarm::STOW_ANTENNAS;
use reachy_motion::postures::NEUTRAL_ANTENNAS;

/// The frame rate every clip document is authored at: the tick rate.
const FRAME_HZ: f64 = reachy_motion::FLOOR_TICK_HZ;

/// A probe's entry blend, milliseconds.
///
/// Zero, and not the format's default, for every probe: the blend is a *weight*
/// ramp over the whole delta a frame carries, so a probe taking the default
/// would compose its first pose a tenth at a time and its first arrival would be
/// a ten-period ramp rather than the step or the stated ramp the table says.
const BLEND_IN_MS: u32 = 0;

/// One antenna pose a probe visits, as a delta over the rest lean.
///
/// A document carries deltas over the base posture it is played on, and the
/// base holds the antennas at [`NEUTRAL_ANTENNAS`]; so the raised pose is the
/// zero delta and every other pose is its constant less that lean.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Pose {
    /// The base itself: antennas raised, zero delta.
    Up,
    /// Outboard horizontal, both antennas pointed away from the head.
    Sides,
    /// The fold the stow puts them in.
    Down,
    /// Half of the way to the fold, which is a goal a reversal can turn on
    /// without an arrival under it.
    HalfDown,
}

impl Pose {
    /// The antenna deltas, right then left, radians.
    #[must_use]
    pub fn antennas(self) -> [f64; 2] {
        let delta = |target: [f64; 2]| {
            [
                target[0] - NEUTRAL_ANTENNAS[0],
                target[1] - NEUTRAL_ANTENNAS[1],
            ]
        };
        match self {
            Self::Up => [0.0, 0.0],
            Self::Sides => delta(ANTENNA_OUTBOARD),
            Self::Down => delta(STOW_ANTENNAS),
            Self::HalfDown => {
                let down = delta(STOW_ANTENNAS);
                [down[0] / 2.0, down[1] / 2.0]
            }
        }
    }
}

/// One stretch of a probe's frame track.
#[derive(Clone, Copy, Debug)]
pub enum Segment {
    /// `frames` copies of one pose. Reached in one frame from wherever the
    /// track stood, which is the step the instrument is made of.
    Hold {
        /// Where the track stands for the whole segment.
        pose: Pose,
        /// How many frames it stands there.
        frames: usize,
    },
    /// A linear walk in the antenna angle, `from` inclusive and `to` exclusive.
    Ramp {
        /// Where the walk starts; the frame it emits first.
        from: Pose,
        /// Where it is heading. The next segment emits this pose.
        to: Pose,
        /// How many frames the walk occupies.
        frames: usize,
    },
}

impl Segment {
    /// The pose the segment's first frame carries.
    fn opens_on(self) -> Pose {
        match self {
            Self::Hold { pose, .. } => pose,
            Self::Ramp { from, .. } => from,
        }
    }

    /// The pose the segment leaves the track heading for, if it did not arrive.
    ///
    /// `None` for a hold, which stands where it is; `Some(to)` for a ramp, whose
    /// last frame is one step short and whose successor owes that pose.
    fn owes(self) -> Option<Pose> {
        match self {
            Self::Hold { .. } => None,
            Self::Ramp { to, .. } => Some(to),
        }
    }

    /// How many frames it emits.
    fn frames(self) -> usize {
        match self {
            Self::Hold { frames, .. } | Self::Ramp { frames, .. } => frames,
        }
    }
}

/// One probe document: what it is called, what it is for, and its frame track.
#[derive(Clone, Copy, Debug)]
pub struct Probe {
    /// The library name, which is also the document's path under the clips
    /// directory: `<clips>/<name>.json`.
    pub name: &'static str,
    /// What the instrument is for, carried into the document's `description`.
    pub description: &'static str,
    /// The one channel the probe drives.
    pub channel: Channel,
    /// The frame track, in order.
    pub segments: &'static [Segment],
}

impl Probe {
    /// The antenna delta of every frame, in order.
    ///
    /// # Errors
    ///
    /// If the table does not chain: a ramp whose successor does not open on the
    /// pose it was heading for never emits that pose, and a track ending on a
    /// ramp stops one step short of its own goal.
    pub fn antenna_frames(&self) -> anyhow::Result<Vec<[f64; 2]>> {
        if self.channel != Channel::Antennas {
            bail!(
                "{}: only the antenna channel has a pose set today",
                self.name
            );
        }
        if self.segments.is_empty() {
            bail!("{}: a probe with no segments has no frames", self.name);
        }
        let mut frames = Vec::new();
        let mut owed: Option<Pose> = None;
        for (index, segment) in self.segments.iter().enumerate() {
            if segment.frames() == 0 {
                bail!("{}: segment {index} emits no frames", self.name);
            }
            if let Some(owed) = owed
                && owed != segment.opens_on()
            {
                bail!(
                    "{}: segment {index} opens on {:?} where the ramp before it was heading for \
                     {owed:?}, which no frame then carries",
                    self.name,
                    segment.opens_on()
                );
            }
            owed = segment.owes();
            match *segment {
                Segment::Hold { pose, frames: held } => {
                    frames.extend(std::iter::repeat_n(pose.antennas(), held));
                }
                Segment::Ramp {
                    from,
                    to,
                    frames: steps,
                } => {
                    let (from, to) = (from.antennas(), to.antennas());
                    #[expect(
                        clippy::cast_precision_loss,
                        reason = "a probe is hundreds of frames, exact as a double"
                    )]
                    for step in 0..steps {
                        let fraction = step as f64 / steps as f64;
                        frames.push([
                            from[0] + (to[0] - from[0]) * fraction,
                            from[1] + (to[1] - from[1]) * fraction,
                        ]);
                    }
                }
            }
        }
        if let Some(owed) = owed {
            bail!(
                "{}: the track ends on a ramp heading for {owed:?}, one step short of it",
                self.name
            );
        }
        Ok(frames)
    }

    /// The document, as the JSON committed under `cogs/clips/probe/`.
    ///
    /// The format's own [`ClipDoc`], serialised: no key of it is spelled here,
    /// so a key the format renames or gains is a compile error in this table
    /// rather than a document the library emit refuses at the bench.
    ///
    /// Laid out one frame per line, with every number as `serde_json` writes
    /// one — the shortest decimal that reads back to the same bits — so a
    /// regenerated document differs from the committed one exactly where a pose
    /// or a segment moved.
    ///
    /// # Errors
    ///
    /// Whatever [`Probe::antenna_frames`] refuses.
    pub fn document(&self) -> anyhow::Result<String> {
        let frames = self.antenna_frames()?;
        Ok(render(&ClipDoc {
            version: FORMAT_VERSION,
            kind: CLIP_KIND.to_owned(),
            name: self.name.to_owned(),
            description: Some(self.description.to_owned()),
            channels: vec![self.channel],
            frame_hz: FRAME_HZ,
            blend_in_ms: Some(BLEND_IN_MS),
            blend_out_ms: None,
            frames: frames
                .iter()
                .map(|frame| FrameDoc {
                    antennas: Some(*frame),
                    ..FrameDoc::default()
                })
                .collect(),
        }))
    }

    /// Where the document belongs, under a clips directory.
    #[must_use]
    pub fn path(&self, clips: &std::path::Path) -> std::path::PathBuf {
        clips.join(format!("{}.json", self.name))
    }
}

/// A clip document as the tree commits one: `serde`'s own JSON, laid out with
/// the frame track one frame per line.
///
/// The layout is what makes a committed document reviewable — 775 frames on one
/// line is a diff nobody reads — and it is all this function decides. Every key
/// and every value comes from the format's own serialisation, so the two are
/// never spelled twice.
fn render(doc: &ClipDoc) -> String {
    let value = serde_json::to_value(doc).expect("a clip document is JSON");
    let fields = value
        .as_object()
        .expect("a clip document is a JSON object")
        .clone();
    let mut out = String::from("{\n");
    for (index, (key, field)) in fields.iter().enumerate() {
        let comma = if index + 1 == fields.len() { "" } else { "," };
        let key = serde_json::to_string(key).expect("a key is JSON");
        match field.as_array().filter(|_| key == "\"frames\"") {
            Some(frames) => {
                let _ = writeln!(out, "  {key}: [");
                for (index, frame) in frames.iter().enumerate() {
                    let inner = if index + 1 == frames.len() { "" } else { "," };
                    let _ = writeln!(out, "    {frame}{inner}");
                }
                let _ = writeln!(out, "  ]{comma}");
            }
            None => {
                let _ = writeln!(out, "  {key}: {field}{comma}");
            }
        }
    }
    out.push_str("}\n");
    out
}

/// Every probe document the tree carries.
///
/// The two step probes are the stillness instruments: three poses off the raised
/// base, each reached in one frame and held 6.5 s — the watch's shortest
/// judgeable hold and half a second — in the two orders, so that between them
/// every arrival (from up, from sides, from down) is judged once.
///
/// The sweep is the residual instrument, and the two do not overlap: a step
/// probe holds still after a hard arrival, where the sweep is moving almost
/// throughout and its one long hold is there only so a run of it has a judged
/// hold at all.
pub const PROBES: &[Probe] = &[
    Probe {
        name: "probe/antenna-step-a",
        description: "antennas alone, up to sides to down to up: each pose is reached in one \
                      frame and held 6.5 s, so the servo's own profile generator is the whole of \
                      the move and the arrival is a hard stop",
        channel: Channel::Antennas,
        segments: &[
            Segment::Hold {
                pose: Pose::Up,
                frames: 1,
            },
            Segment::Hold {
                pose: Pose::Sides,
                frames: 325,
            },
            Segment::Hold {
                pose: Pose::Down,
                frames: 325,
            },
            Segment::Hold {
                pose: Pose::Up,
                frames: 325,
            },
        ],
    },
    Probe {
        name: "probe/antenna-step-b",
        description: "antennas alone, up to down to sides to up: the same one-frame steps as \
                      antenna-step-a over the other order, which is where the up-to-down and \
                      sides-to-down arrivals come from",
        channel: Channel::Antennas,
        segments: &[
            Segment::Hold {
                pose: Pose::Up,
                frames: 1,
            },
            Segment::Hold {
                pose: Pose::Down,
                frames: 325,
            },
            Segment::Hold {
                pose: Pose::Sides,
                frames: 325,
            },
            Segment::Hold {
                pose: Pose::Up,
                frames: 325,
            },
        ],
    },
    Probe {
        name: "probe/antenna-sweep",
        description: "antennas alone, the library's antenna stress in miniature: streamed ramps \
                      from half the fast pair's cap to well under the shipped one, a reversal at \
                      an arrival and a reversal mid-move, the outboard arc, and one head-still \
                      hold at the end so a run of it has a hold to judge",
        channel: Channel::Antennas,
        segments: &[
            // Streamed content at about half the fast pair's cap, which
            // saturates the shipped (20, 50) pair outright.
            Segment::Ramp {
                from: Pose::Up,
                to: Pose::Down,
                frames: 20,
            },
            Segment::Hold {
                pose: Pose::Down,
                frames: 50,
            },
            // Content at the fast pair's cap.
            Segment::Ramp {
                from: Pose::Down,
                to: Pose::Up,
                frames: 10,
            },
            Segment::Hold {
                pose: Pose::Up,
                frames: 50,
            },
            // A reversal at the arrival: the goal turns on the frame the track
            // reaches the fold, with no hold under it.
            Segment::Ramp {
                from: Pose::Up,
                to: Pose::Down,
                frames: 10,
            },
            Segment::Ramp {
                from: Pose::Down,
                to: Pose::Up,
                frames: 10,
            },
            Segment::Hold {
                pose: Pose::Up,
                frames: 50,
            },
            // A reversal mid-move: the turn is at a goal the track was never
            // going to arrive at.
            Segment::Ramp {
                from: Pose::Up,
                to: Pose::HalfDown,
                frames: 10,
            },
            Segment::Ramp {
                from: Pose::HalfDown,
                to: Pose::Up,
                frames: 10,
            },
            Segment::Hold {
                pose: Pose::Up,
                frames: 50,
            },
            // The outboard arc, as the step probes take it, streamed instead of
            // stepped.
            Segment::Ramp {
                from: Pose::Up,
                to: Pose::Sides,
                frames: 15,
            },
            Segment::Ramp {
                from: Pose::Sides,
                to: Pose::Down,
                frames: 15,
            },
            Segment::Hold {
                pose: Pose::Down,
                frames: 50,
            },
            // Content near the shipped pair's own cap: what (20, 50) can just
            // follow.
            Segment::Ramp {
                from: Pose::Down,
                to: Pose::Up,
                frames: 100,
            },
            // The one judged hold, 6.5 s like a step probe's.
            Segment::Hold {
                pose: Pose::Up,
                frames: 325,
            },
        ],
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    /// The poses are the constants, as differences over the rest lean. Stated
    /// here so that moving a constant moves the documents and this case
    /// together, rather than either one alone.
    #[test]
    fn every_pose_is_a_difference_of_the_tree_s_own_constants() {
        assert_eq!(Pose::Up.antennas(), [0.0, 0.0]);
        assert_eq!(
            Pose::Sides.antennas(),
            [
                ANTENNA_OUTBOARD[0] - NEUTRAL_ANTENNAS[0],
                ANTENNA_OUTBOARD[1] - NEUTRAL_ANTENNAS[1]
            ]
        );
        assert_eq!(
            Pose::Down.antennas(),
            [
                STOW_ANTENNAS[0] - NEUTRAL_ANTENNAS[0],
                STOW_ANTENNAS[1] - NEUTRAL_ANTENNAS[1]
            ]
        );
        assert_eq!(
            Pose::HalfDown.antennas(),
            [
                Pose::Down.antennas()[0] / 2.0,
                Pose::Down.antennas()[1] / 2.0
            ]
        );
    }

    /// Every probe starts and ends on the base, so the entry composes nothing
    /// on its first frame and the exit blend has nothing left to unwind.
    #[test]
    fn every_probe_opens_and_closes_on_the_base() {
        for probe in PROBES {
            let frames = probe.antenna_frames().expect("the table chains");
            assert_eq!(frames[0], Pose::Up.antennas(), "{}", probe.name);
            assert_eq!(
                frames[frames.len() - 1],
                Pose::Up.antennas(),
                "{}",
                probe.name
            );
        }
    }

    /// A segment's frame count is what the table says, summed.
    #[test]
    fn a_probe_emits_the_frames_its_segments_state() {
        for probe in PROBES {
            let stated: usize = probe.segments.iter().map(|segment| segment.frames()).sum();
            let frames = probe.antenna_frames().expect("the table chains");
            assert_eq!(frames.len(), stated, "{}", probe.name);
        }
        let sweep = PROBES
            .iter()
            .find(|probe| probe.name == "probe/antenna-sweep")
            .expect("the sweep is in the table");
        assert_eq!(
            sweep.antenna_frames().expect("the table chains").len(),
            775,
            "15.5 s at the tick rate"
        );
    }

    /// The two step probes are steps and not ramps: a base frame, then three
    /// poses each held long enough for the watch to judge.
    #[test]
    fn the_step_probes_step_to_three_held_poses() {
        for name in ["probe/antenna-step-a", "probe/antenna-step-b"] {
            let probe = PROBES
                .iter()
                .find(|probe| probe.name == name)
                .unwrap_or_else(|| panic!("{name} is in the table"));
            assert!(
                probe
                    .segments
                    .iter()
                    .all(|segment| matches!(segment, Segment::Hold { .. })),
                "{name} is stepped, not ramped"
            );
            let frames = probe.antenna_frames().expect("the table chains");
            assert_eq!(frames.len(), 1 + 3 * 325, "{name}");
        }
    }

    /// A ramp emits its opening pose and not its goal, so two ramps back to
    /// back turn on one frame and a ramp into a hold arrives exactly once.
    #[test]
    fn a_ramp_carries_its_opening_pose_and_leaves_its_goal_to_what_follows() {
        let probe = Probe {
            name: "probe/case",
            description: "",
            channel: Channel::Antennas,
            segments: &[
                Segment::Ramp {
                    from: Pose::Up,
                    to: Pose::Down,
                    frames: 2,
                },
                Segment::Ramp {
                    from: Pose::Down,
                    to: Pose::Up,
                    frames: 2,
                },
                Segment::Hold {
                    pose: Pose::Up,
                    frames: 1,
                },
            ],
        };
        let down = Pose::Down.antennas();
        let frames = probe.antenna_frames().expect("the table chains");
        assert_eq!(
            frames,
            vec![
                [0.0, 0.0],
                [down[0] / 2.0, down[1] / 2.0],
                down,
                [down[0] / 2.0, down[1] / 2.0],
                [0.0, 0.0],
            ]
        );
    }

    /// A ramp whose successor opens somewhere else never emits the pose it was
    /// heading for: refused, because nothing in the table would show it.
    #[test]
    fn a_ramp_whose_successor_starts_elsewhere_is_refused() {
        let probe = Probe {
            name: "probe/case",
            description: "",
            channel: Channel::Antennas,
            segments: &[
                Segment::Ramp {
                    from: Pose::Up,
                    to: Pose::Down,
                    frames: 2,
                },
                Segment::Hold {
                    pose: Pose::Sides,
                    frames: 1,
                },
            ],
        };
        let refused = probe.antenna_frames().expect_err("the chain is broken");
        assert!(format!("{refused:#}").contains("no frame then carries"));
    }

    /// A track that ends on a ramp stops one step short of its own goal.
    #[test]
    fn a_track_ending_on_a_ramp_is_refused() {
        let probe = Probe {
            name: "probe/case",
            description: "",
            channel: Channel::Antennas,
            segments: &[Segment::Ramp {
                from: Pose::Up,
                to: Pose::Down,
                frames: 2,
            }],
        };
        let refused = probe.antenna_frames().expect_err("the track is short");
        assert!(format!("{refused:#}").contains("one step short"));
    }

    /// A channel with no pose set is refused rather than written as antennas.
    #[test]
    fn a_probe_on_a_channel_with_no_poses_is_refused() {
        let probe = Probe {
            name: "probe/case",
            description: "",
            channel: Channel::BodyYaw,
            segments: &[Segment::Hold {
                pose: Pose::Up,
                frames: 1,
            }],
        };
        let refused = probe.antenna_frames().expect_err("no yaw poses");
        assert!(format!("{refused:#}").contains("only the antenna channel"));
    }

    /// The document's shape: the keys a clip document carries, in the order the
    /// format's own serialisation states them, the blend the probes pin, and one
    /// frame per line.
    #[test]
    fn a_document_states_the_clip_keys_and_one_frame_a_line() {
        let probe = Probe {
            name: "probe/case",
            description: "a \"quoted\" description",
            channel: Channel::Antennas,
            segments: &[Segment::Hold {
                pose: Pose::Up,
                frames: 2,
            }],
        };
        let document = probe.document().expect("the table chains");
        assert_eq!(
            document,
            "{\n  \"blend_in_ms\": 0,\n  \"channels\": [\"antennas\"],\n  \
             \"description\": \"a \\\"quoted\\\" description\",\n  \"frame_hz\": 50.0,\n  \
             \"frames\": [\n    {\"antennas\":[0.0,0.0]},\n    {\"antennas\":[0.0,0.0]}\n  \
             ],\n  \"kind\": \"clip\",\n  \"name\": \"probe/case\",\n  \"version\": 1\n}\n"
        );
    }

    /// Every name is under the probe prefix and lands where the emitter walks.
    #[test]
    fn a_probe_is_written_where_the_walk_reads_it() {
        for probe in PROBES {
            assert!(probe.name.starts_with("probe/"), "{}", probe.name);
            assert_eq!(
                probe.path(std::path::Path::new("/tmp/clips")),
                std::path::PathBuf::from(format!("/tmp/clips/{}.json", probe.name))
            );
        }
    }
}
