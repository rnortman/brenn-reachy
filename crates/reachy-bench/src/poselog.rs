//! The pose log's line schema: the one declaration both ends of it read.
//!
//! The pose stream is an inter-process wire format. The bench writes it to its
//! console at 50 Hz while a person moves the machine by hand, and the offline
//! session analyzer reads that file back after a fetch — two binaries in two
//! packages, on one contract. So the contract is one set of types with both
//! serde halves derived, rather than a writer's structs and a reader's enum that
//! happen to spell the same seven words: a second declaration diverges on the
//! first field nobody renamed twice, and the divergence is invisible until a
//! fetched session reads as a recording with no samples in it. That costs a
//! hardware round trip to diagnose, which is the expensive thing this whole
//! feature exists to spend well.
//!
//! The sampling grid lives here for the same reason. The writer paces on it and
//! the reader judges a sample step against it, and a reader restating the
//! literal would report every step of a stream recorded on another period as a
//! hole.
//!
//! Counts, not radians: a count is what the servo said, and the conversion is
//! one linear unwrapped map a reader applies itself.

use core::time::Duration;

use reachy_motion::segments::SegmentConfig;
use serde::{Deserialize, Serialize};

use reachy_motion::joints::ROW_COUNT;

/// The grid the pose log samples on, milliseconds.
pub const POSE_LOG_PERIOD_MS: u64 = 20;

/// The grid the pose log samples on: 50 Hz.
///
/// The grouped read is about 1.6 ms of wire at 1 Mbaud plus the host's
/// turnaround, so a 20 ms period has room for nothing else and the stream needs
/// nothing else. It is also the frame rate a clip document carries, so a
/// recorded stretch is one frame per sample with no resampling between the
/// recording and the clip.
pub const POSE_LOG_PERIOD: Duration = Duration::from_millis(POSE_LOG_PERIOD_MS);

/// One line of the pose stream, whichever kind it is.
///
/// Internally tagged on `kind`, which is the spelling on the wire: a writer
/// serializes a variant and a reader matches on one, so neither end spells the
/// seven words itself.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PoseLine {
    /// What is being read, over what, on what grid, and under which thresholds
    /// the live still/moving notes are cut.
    Started(StartedLine),
    /// One reading of the nine, as counts.
    Sample(SampleLine),
    /// A sample that landed more than a period behind the grid.
    Late(LateLine),
    /// The stream became still, as of the stamp the line carries.
    Still(StateLine),
    /// The stream started moving, as of the stamp the line carries.
    Moving(StateLine),
    /// The closing line.
    Ended(EndedLine),
    /// A run that refused before it sampled anything.
    Refused(RefusedLine),
}

/// The opening line.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StartedLine {
    /// When the run opened, epoch nanoseconds.
    pub t_ns: i64,
    /// The port the stream was read over.
    pub device: String,
    /// The servo ids, in bus-row order.
    pub ids: [u8; ROW_COUNT],
    /// The grid the samples were taken on, milliseconds.
    pub period_ms: u64,
    /// What Torque Enable read on each servo before the first sample: all
    /// zeros, or the run refused.
    pub torque: [u8; ROW_COUNT],
    /// The thresholds the live notes were cut on.
    pub segmenter: SegmenterLine,
}

/// A segmenter configuration, in the units the flags and the recorder speak.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct SegmenterLine {
    /// The difference baseline, milliseconds.
    pub speed_window_ms: u64,
    /// Below this activity every joint is standing, rad/s.
    pub still_rad_s: f64,
    /// At or above this activity the stream is moving, rad/s.
    pub moving_rad_s: f64,
    /// The shortest quiet run that is a hold, milliseconds.
    pub min_still_ms: u64,
}

impl SegmenterLine {
    /// How `cfg` is written down.
    #[must_use]
    pub fn of(cfg: SegmentConfig) -> Self {
        Self {
            speed_window_ms: millis(cfg.speed_window),
            still_rad_s: cfg.still_rad_per_s,
            moving_rad_s: cfg.moving_rad_per_s,
            min_still_ms: millis(cfg.min_still),
        }
    }
}

/// One reading of the nine, as counts. A row that did not answer is null.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SampleLine {
    /// Taken immediately before the instruction frame went out, epoch
    /// nanoseconds.
    pub t_ns: i64,
    /// The same instant on the clock the sampling grid is paced on:
    /// nanoseconds since the run began, monotonic.
    ///
    /// Two stamps per reading, one from each clock, because neither alone says
    /// what happened between two samples. The wall stamp is the join key
    /// against the voice host's, and it can step: the unit boots from RAM and
    /// sets its clock from the network, so a session started before the first
    /// sync sees one large jump. The grid clock does not step, so the pair is
    /// what tells a reader whether a widened step was the clock moving or the
    /// stream missing a stretch — which are different facts about what the head
    /// was doing, and only one of them is recoverable.
    ///
    /// Required, not defaulted: recorder and reader ship in one payload and
    /// this schema binds by byte equality, so a stream without it is a stream
    /// from another build and reads as one.
    pub mono_ns: i64,
    /// How long the grouped exchange took, microseconds.
    pub read_us: u64,
    /// The nine readings in bus-row order; a row that did not answer is null.
    pub counts: [Option<i32>; ROW_COUNT],
}

/// A sample that landed more than a period behind the grid.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct LateLine {
    /// When the note was written, epoch nanoseconds.
    pub t_ns: i64,
    /// How far behind the grid the loop had fallen, microseconds.
    pub behind_us: u64,
}

/// The stream's state changed, as of `t0_ns`.
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct StateLine {
    /// When the state began — retroactively, for a hold, which is why the line
    /// is written later than the stamp it carries.
    pub t0_ns: i64,
}

/// The closing line.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EndedLine {
    /// When the run closed, epoch nanoseconds.
    pub t_ns: i64,
    /// How many samples went out.
    pub samples: usize,
    /// How many `late` notes went out.
    pub late: usize,
    /// What Torque Enable read on each servo after the run, or null for a servo
    /// that no longer answered.
    pub torque: [Option<u8>; ROW_COUNT],
    /// How the run ended.
    pub cause: PoseLogEnd,
    /// What ended it, for a run that ended on a failure rather than on the
    /// operator or the cap; absent otherwise.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// A run that refused, as a line of the stream it never wrote.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RefusedLine {
    /// When the refusal was written, epoch nanoseconds.
    pub t_ns: i64,
    /// What the refusal said.
    pub error: String,
}

/// Why a pose log stopped.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum PoseLogEnd {
    /// SIGINT or SIGTERM: the operator ended the session.
    Signal,
    /// The `--seconds` cap ran out.
    Seconds,
    /// Something failed under a run that had already started streaming — the
    /// bus went away — and the samples ahead of it are a recording. What
    /// failed is the line's `error`.
    ///
    /// Distinct from a `refused` line, which is a run that never sampled
    /// anything: a session whose adapter hiccuped after twenty good minutes is
    /// not a session that refused to run, and a reader told otherwise throws
    /// the twenty minutes away.
    Aborted,
}

/// A duration as whole milliseconds, saturating rather than wrapping on a span
/// no clock will ever produce.
fn millis(span: Duration) -> u64 {
    u64::try_from(span.as_millis()).unwrap_or(u64::MAX)
}

/// Microseconds as the line schema carries them, from whatever width the clock
/// answered in.
#[must_use]
pub fn micros(span: Duration) -> u64 {
    u64::try_from(span.as_micros()).unwrap_or(u64::MAX)
}

/// Nanoseconds as the line schema carries them, from whatever width the clock
/// answered in.
///
/// Saturating: 292 years of monotonic uptime is not a session, and a stamp that
/// wrapped would read as a clock that went backwards.
#[must_use]
pub fn nanos(span: Duration) -> i64 {
    i64::try_from(span.as_nanos()).unwrap_or(i64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The round trip both ends stand on: what a writer serializes is what a
    /// reader matches, field name for field name. A rename on either side of a
    /// second declaration would have been invisible here; with one declaration
    /// it cannot happen, and this holds the wire spelling itself steady.
    #[test]
    fn every_kind_round_trips_through_its_own_wire_spelling() {
        let lines = [
            PoseLine::Started(StartedLine {
                t_ns: 7,
                device: "/dev/ttyAMA3".to_owned(),
                ids: [10, 11, 12, 13, 14, 15, 16, 17, 18],
                period_ms: POSE_LOG_PERIOD_MS,
                torque: [0; ROW_COUNT],
                segmenter: SegmenterLine::of(SegmentConfig::default()),
            }),
            PoseLine::Sample(SampleLine {
                t_ns: 27,
                mono_ns: 20_000_000,
                read_us: 1700,
                counts: [
                    Some(2048),
                    None,
                    Some(1),
                    Some(2),
                    Some(3),
                    Some(4),
                    Some(5),
                    Some(6),
                    Some(7),
                ],
            }),
            PoseLine::Late(LateLine {
                t_ns: 47,
                behind_us: 40_000,
            }),
            PoseLine::Still(StateLine { t0_ns: 67 }),
            PoseLine::Moving(StateLine { t0_ns: 87 }),
            PoseLine::Ended(EndedLine {
                t_ns: 107,
                samples: 5,
                late: 1,
                torque: [Some(0); ROW_COUNT],
                cause: PoseLogEnd::Signal,
                error: None,
            }),
            PoseLine::Refused(RefusedLine {
                t_ns: 127,
                error: "servo 12 holds torque".to_owned(),
            }),
            PoseLine::Ended(EndedLine {
                t_ns: 147,
                samples: 900,
                late: 0,
                torque: [None; ROW_COUNT],
                cause: PoseLogEnd::Aborted,
                error: Some("servo 12 stopped answering".to_owned()),
            }),
        ];

        let kinds = [
            "started", "sample", "late", "still", "moving", "ended", "refused", "ended",
        ];
        for (line, kind) in lines.iter().zip(kinds) {
            let text = serde_json::to_string(line).expect("a record of numbers and strings");
            assert!(
                text.contains(&format!("\"kind\":\"{kind}\"")),
                "{kind}: {text}"
            );
            let read: PoseLine = serde_json::from_str(&text).expect("its own spelling");
            let again = serde_json::to_string(&read).expect("a record of numbers and strings");
            assert_eq!(text, again, "{kind}");
        }
    }

    /// The three ways a run ends, as the reader sees them: the two clean ones
    /// carry no failure, and the third carries what failed.
    #[test]
    fn a_clean_close_carries_no_error_and_an_abort_carries_one() {
        let clean = serde_json::to_string(&PoseLine::Ended(EndedLine {
            t_ns: 7,
            samples: 50,
            late: 0,
            torque: [Some(0); ROW_COUNT],
            cause: PoseLogEnd::Seconds,
            error: None,
        }))
        .expect("a record of numbers and strings");
        assert!(clean.contains("\"cause\":\"seconds\""), "{clean}");
        assert!(!clean.contains("error"), "{clean}");

        let aborted = serde_json::to_string(&PoseLine::Ended(EndedLine {
            t_ns: 7,
            samples: 50,
            late: 0,
            torque: [Some(0); ROW_COUNT],
            cause: PoseLogEnd::Aborted,
            error: Some("the port went away".to_owned()),
        }))
        .expect("a record of numbers and strings");
        assert!(aborted.contains("\"cause\":\"aborted\""), "{aborted}");
        assert!(aborted.contains("the port went away"), "{aborted}");
    }

    /// The grid stamp is part of the contract, not an optional extra: a stream
    /// that carries only the wall clock cannot be told apart from one whose
    /// clock stepped, and reading it as if it could is how a hold nobody held
    /// gets drafted into a clip. A pre-append stream is refused as what it is —
    /// a stream from another build.
    #[test]
    fn a_sample_line_without_the_grid_stamp_is_refused() {
        let without = r#"{"kind":"sample","t_ns":27,"read_us":1700,"counts":[1,2,3,4,5,6,7,8,9]}"#;
        let error = serde_json::from_str::<PoseLine>(without).expect_err("one clock is not two");
        assert!(error.to_string().contains("mono_ns"), "{error}");

        let with = r#"{"kind":"sample","t_ns":27,"mono_ns":20000000,"read_us":1700,"counts":[1,2,3,4,5,6,7,8,9]}"#;
        let read: PoseLine = serde_json::from_str(with).expect("both clocks");
        let PoseLine::Sample(sample) = read else {
            panic!("a sample line reads as a sample");
        };
        assert_eq!(sample.mono_ns, 20_000_000);
    }

    /// The grid stamp's conversion, including the end it saturates at.
    ///
    /// A span no clock will ever answer with is pinned because the alternative
    /// spelling — a cast — wraps to a negative, and a grid stamp that went
    /// backwards reads downstream as a clock step on every sample after it.
    #[test]
    fn a_grid_stamp_is_whole_nanoseconds_and_saturates_rather_than_wrapping() {
        assert_eq!(nanos(Duration::from_millis(20)), 20_000_000);
        assert_eq!(nanos(Duration::ZERO), 0);
        assert_eq!(nanos(Duration::new(u64::MAX, 0)), i64::MAX);
        assert_eq!(micros(Duration::from_millis(20)), 20_000);
        assert_eq!(micros(Duration::new(u64::MAX, 0)), u64::MAX);
    }

    #[test]
    fn the_segmenter_line_is_the_configuration_in_milliseconds() {
        let cfg = SegmentConfig::default();
        let written = SegmenterLine::of(cfg);
        assert_eq!(written.speed_window_ms, 100);
        assert_eq!(written.min_still_ms, 500);
        assert!((written.still_rad_s - cfg.still_rad_per_s).abs() < f64::EPSILON);
        assert!((written.moving_rad_s - cfg.moving_rad_per_s).abs() < f64::EPSILON);
    }
}
