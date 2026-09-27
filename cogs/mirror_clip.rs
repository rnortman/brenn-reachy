//! `mirror-clip` — an offline authoring tool that writes the left–right mirror
//! of a clip document.
//!
//! The map is reflection through the head's x–z plane: the head translation's
//! y is negated, the rotation quaternion `(w, x, y, z)` becomes `(w, −x, y, −z)`,
//! body yaw is negated, and the antenna pair `[right, left]` becomes
//! `[−left, −right]`. A base is carried across the same way: a numeric base is
//! mapped, and a named one is kept only when it is its own mirror, which is
//! `neutral`. With `--copy-head` the head channel is copied unmirrored and only
//! the antennas and body yaw are mirrored.
//!
//! Run as
//! `bazel run //cogs:mirror_clip -- --in <clip.json> --out <mirror.json> --name <name> [--copy-head]`,
//! followed by `make library-config`. The output is a committed document,
//! screened like any other when the library is emitted.

#![forbid(unsafe_code)]

use std::path::PathBuf;

use anyhow::{Context as _, bail};

use reachy_clips::format::{
    BaseDoc, ClipDoc, FrameDoc, HeadBaseDoc, NumericBaseDoc, render_document,
};

/// The only named base that is its own mirror.
const SELF_MIRRORED_BASE: &str = "neutral";

/// What happens to the head channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Head {
    /// Mirrored with everything else.
    Mirror,
    /// Carried over unmirrored.
    Copy,
}

/// Exact negation; a zero stays `+0.0`, so an unmoved axis serialises as it
/// was written.
fn flip(v: f64) -> f64 {
    0.0 - v
}

/// A head translation reflected through the x–z plane.
fn mirror_dt([x, y, z]: [f64; 3]) -> [f64; 3] {
    [x, flip(y), z]
}

/// A head rotation reflected through the x–z plane: the roll and yaw
/// components change sign, the pitch does not.
fn mirror_dq([w, x, y, z]: [f64; 4]) -> [f64; 4] {
    [w, flip(x), y, flip(z)]
}

/// The antenna pair swapped side for side, each angle negated.
fn mirror_antennas([right, left]: [f64; 2]) -> [f64; 2] {
    [flip(left), flip(right)]
}

/// A head delta under `head`.
fn head_dt(dt: [f64; 3], head: Head) -> [f64; 3] {
    match head {
        Head::Mirror => mirror_dt(dt),
        Head::Copy => dt,
    }
}

/// A head rotation under `head`.
fn head_dq(dq: [f64; 4], head: Head) -> [f64; 4] {
    match head {
        Head::Mirror => mirror_dq(dq),
        Head::Copy => dq,
    }
}

/// The mirror of `doc`, named `name`.
fn mirrored(doc: &ClipDoc, name: &str, head: Head) -> anyhow::Result<ClipDoc> {
    let base = match &doc.base {
        None => None,
        Some(BaseDoc::Named(pose)) if pose == SELF_MIRRORED_BASE => {
            Some(BaseDoc::Named(pose.clone()))
        }
        Some(BaseDoc::Named(other)) => bail!(
            "`{other}` is not its own mirror; only `{SELF_MIRRORED_BASE}` is, so a clip posed \
             over `{other}` has no mirror over the same base"
        ),
        Some(BaseDoc::Numeric(numeric)) => Some(BaseDoc::Numeric(NumericBaseDoc {
            head: numeric.head.as_ref().map(|base| HeadBaseDoc {
                dt: head_dt(base.dt, head),
                dq: head_dq(base.dq, head),
            }),
            body_yaw: numeric.body_yaw.map(flip),
            antennas: numeric.antennas.map(mirror_antennas),
        })),
    };
    let lead = match head {
        Head::Mirror => format!("Mirror of `{}`", doc.name),
        Head::Copy => format!(
            "Mirror of `{}`, antennas and body yaw only; the head channel is copied unmirrored",
            doc.name
        ),
    };
    let description = match &doc.description {
        Some(original) => format!("{lead}: {original}"),
        None => format!("{lead}."),
    };
    let frames = doc
        .frames
        .iter()
        .map(|frame| FrameDoc {
            dt: frame.dt.map(|dt| head_dt(dt, head)),
            dq: frame.dq.map(|dq| head_dq(dq, head)),
            antennas: frame.antennas.map(mirror_antennas),
            body_yaw: frame.body_yaw.map(flip),
        })
        .collect();
    Ok(ClipDoc {
        version: doc.version,
        kind: doc.kind.clone(),
        name: name.to_owned(),
        base,
        description: Some(description),
        channels: doc.channels.clone(),
        frame_hz: doc.frame_hz,
        blend_in_ms: doc.blend_in_ms,
        blend_out_ms: doc.blend_out_ms,
        frames,
    })
}

/// What the command line asked for.
struct Args {
    input: PathBuf,
    output: PathBuf,
    name: String,
    head: Head,
}

/// The four flags, read by hand.
fn parse_args(mut args: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let (mut input, mut output, mut name, mut head) = (None, None, None, Head::Mirror);
    while let Some(flag) = args.next() {
        let mut value = || {
            args.next()
                .with_context(|| format!("{flag} takes a value and none was given"))
        };
        match flag.as_str() {
            "--in" => input = Some(PathBuf::from(value()?)),
            "--out" => output = Some(PathBuf::from(value()?)),
            "--name" => name = Some(value()?),
            "--copy-head" => head = Head::Copy,
            other => bail!("unknown flag {other}"),
        }
    }
    let input = input.context("--in is required")?;
    let output = output.context("--out is required")?;
    let name = name.context("--name is required")?;
    if input == output {
        bail!(
            "--out is --in ({}): the mirror is written beside its source, never over it",
            input.display()
        );
    }
    Ok(Args {
        input,
        output,
        name,
        head,
    })
}

fn main() -> anyhow::Result<()> {
    let args = parse_args(std::env::args().skip(1))?;
    let text = std::fs::read_to_string(&args.input)
        .with_context(|| format!("reading {}", args.input.display()))?;
    let doc: ClipDoc = serde_json::from_str(&text)
        .with_context(|| format!("{} is not a clip document", args.input.display()))?;
    let mirror = mirrored(&doc, &args.name, args.head)?;
    std::fs::write(&args.output, render_document(&mirror))
        .with_context(|| format!("writing {}", args.output.display()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use reachy_clips::format::Channel;

    fn frame(dt: [f64; 3], dq: [f64; 4], antennas: [f64; 2], body_yaw: f64) -> FrameDoc {
        FrameDoc {
            dt: Some(dt),
            dq: Some(dq),
            antennas: Some(antennas),
            body_yaw: Some(body_yaw),
        }
    }

    /// A document over all three channels, a numeric base carrying every one,
    /// and three frames with nothing zero.
    fn document(description: Option<&str>) -> ClipDoc {
        ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: "source".to_owned(),
            base: Some(BaseDoc::Numeric(NumericBaseDoc {
                head: Some(HeadBaseDoc {
                    dt: [0.01, -0.02, 0.03],
                    dq: [0.98, 0.1, -0.1, 0.12],
                }),
                body_yaw: Some(0.3),
                antennas: Some([-0.2, 0.4]),
            })),
            description: description.map(str::to_owned),
            channels: vec![Channel::Head, Channel::Antennas, Channel::BodyYaw],
            frame_hz: 50.0,
            blend_in_ms: Some(100),
            blend_out_ms: Some(200),
            frames: vec![
                frame([0.1, 0.2, 0.3], [0.9, 0.1, 0.2, 0.3], [0.4, -0.1], 0.25),
                frame(
                    [-0.01, 0.005, 0.02],
                    [0.99, -0.05, 0.03, 0.07],
                    [0.7, 0.2],
                    -0.1,
                ),
                frame(
                    [0.002, -0.004, -0.006],
                    [0.97, 0.2, -0.1, -0.05],
                    [-0.3, -0.6],
                    0.05,
                ),
            ],
        }
    }

    #[test]
    fn one_frame_mirrors_by_the_map() {
        let doc = ClipDoc {
            base: None,
            frames: vec![frame(
                [0.1, 0.2, 0.3],
                [0.9, 0.1, 0.2, 0.3],
                [0.4, -0.1],
                0.25,
            )],
            ..document(None)
        };
        let mirror = mirrored(&doc, "m", Head::Mirror).expect("an overlay mirrors");
        assert_eq!(
            mirror.frames,
            vec![frame(
                [0.1, -0.2, 0.3],
                [0.9, -0.1, 0.2, -0.3],
                [0.1, -0.4],
                -0.25
            )]
        );
    }

    #[test]
    fn mirroring_twice_is_the_identity() {
        let doc = document(None);
        for head in [Head::Mirror, Head::Copy] {
            let once = mirrored(&doc, "m", head).expect("mirrors");
            let twice = mirrored(&once, &doc.name, head).expect("mirrors back");
            assert_eq!(twice.frames, doc.frames, "{head:?}");
            assert_eq!(twice.base, doc.base, "{head:?}");
            assert_eq!(twice.name, doc.name, "{head:?}");
        }
    }

    #[test]
    fn copy_head_leaves_the_head_channel_as_it_was() {
        let doc = document(None);
        let mirror = mirrored(&doc, "m", Head::Copy).expect("mirrors");
        for (mirrored, source) in mirror.frames.iter().zip(&doc.frames) {
            assert_eq!(mirrored.dt, source.dt);
            assert_eq!(mirrored.dq, source.dq);
            assert_eq!(mirrored.antennas, source.antennas.map(mirror_antennas));
            assert_eq!(mirrored.body_yaw, source.body_yaw.map(flip));
        }
        let (Some(BaseDoc::Numeric(mirrored)), Some(BaseDoc::Numeric(source))) =
            (&mirror.base, &doc.base)
        else {
            panic!("a numeric base stays numeric");
        };
        assert_eq!(mirrored.head, source.head);
        assert_eq!(mirrored.body_yaw, source.body_yaw.map(flip));
        assert_eq!(mirrored.antennas, source.antennas.map(mirror_antennas));
    }

    #[test]
    fn a_named_base_is_kept_only_when_it_is_neutral() {
        let neutral = ClipDoc {
            base: Some(BaseDoc::Named("neutral".to_owned())),
            ..document(None)
        };
        let mirror = mirrored(&neutral, "m", Head::Mirror).expect("neutral is its own mirror");
        assert_eq!(mirror.base, Some(BaseDoc::Named("neutral".to_owned())));

        let hello = ClipDoc {
            base: Some(BaseDoc::Named("hello".to_owned())),
            ..document(None)
        };
        let error = mirrored(&hello, "m", Head::Mirror).expect_err("hello is not symmetric");
        assert!(error.to_string().contains("`hello`"), "{error}");
    }

    #[test]
    fn the_description_names_the_source() {
        let cases = [
            (Head::Mirror, None, "Mirror of `source`."),
            (Head::Mirror, Some("a wave"), "Mirror of `source`: a wave"),
            (
                Head::Copy,
                None,
                "Mirror of `source`, antennas and body yaw only; the head channel is copied \
                 unmirrored.",
            ),
            (
                Head::Copy,
                Some("a wave"),
                "Mirror of `source`, antennas and body yaw only; the head channel is copied \
                 unmirrored: a wave",
            ),
        ];
        for (head, description, wanted) in cases {
            let mirror = mirrored(&document(description), "m", head).expect("mirrors");
            assert_eq!(mirror.description.as_deref(), Some(wanted), "{head:?}");
        }
    }

    #[test]
    fn the_command_line_is_read_and_refused_by_flag() {
        let argv = |words: &[&str]| {
            words
                .iter()
                .map(|word| (*word).to_owned())
                .collect::<Vec<_>>()
        };
        let args = parse_args(
            argv(&[
                "--in",
                "a.json",
                "--out",
                "b.json",
                "--name",
                "b",
                "--copy-head",
            ])
            .into_iter(),
        )
        .expect("a full command line");
        assert_eq!(args.input, PathBuf::from("a.json"));
        assert_eq!(args.output, PathBuf::from("b.json"));
        assert_eq!(args.name, "b");
        assert_eq!(args.head, Head::Copy);

        for (words, named) in [
            (argv(&["--in", "a.json", "--out", "b.json"]), "--name"),
            (argv(&["--in", "a.json", "--name", "b", "--out"]), "--out"),
            (argv(&["--bogus"]), "--bogus"),
            (
                argv(&["--in", "a.json", "--out", "a.json", "--name", "b"]),
                "--out",
            ),
        ] {
            let error = parse_args(words.into_iter()).err().expect("refused");
            assert!(error.to_string().contains(named), "{error}");
        }
    }
}
