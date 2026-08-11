//! The per-period trace as a file: one row per control period, CSV.
//!
//! What a run's summary carries is aggregate — periods, worst lag, when the
//! machine arrived. The question that needs the rest is what the motion looked
//! like: where each joint was at every period, against the goal it was being
//! held to. That is a velocity profile in the only form this loop can honestly
//! produce one, sampled at the rate the servos were actually read at.
//! [`metrics`] is the reader that differences it.
//!
//! Written once per run, not once per period. The loop keeps its samples in
//! memory and this renders them afterwards, so nothing in a control period ever
//! waits on a file. For the same reason the destination belongs on a memory
//! filesystem — `/run` on the machine this drives — and never on the device's
//! flash.

pub mod metrics;

use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::Path;

use reachy_motion::{JointId, JointSet, JointVector};

use crate::pump::TickSample;

/// How many bytes of a file's tail are read to find the last run's number.
///
/// A row is a few hundred bytes, so this reaches back over several of them: all
/// that is needed is the complete rows at the end, and reading the whole file
/// to find them would mean reading every period of every run already in it.
const TAIL_BYTES: u64 = 4096;

/// One joint's column key.
///
/// Spelled out rather than taken from [`JointId`]'s own rendering: those are
/// display names for an operator reading a line of prose, and a column heading
/// is a key something downstream indexes by. Underscores and no spaces, so a
/// reader that splits on commas gets names it can use as identifiers.
///
/// Derived per joint rather than kept as a parallel array, so the compiler owns
/// the correspondence between a heading and the angle written under it: the
/// header is built by walking [`JointId::ALL`], which is the same bus order
/// [`JointVector::joints`] writes in.
fn column(joint: JointId) -> String {
    match joint {
        JointId::BodyYaw => "body_yaw".to_string(),
        // 1-based, as the servos and the operator-facing messages number them.
        JointId::Leg(leg) => format!("leg{}", u16::from(leg) + 1),
        JointId::AntennaRight => "antenna_right".to_string(),
        JointId::AntennaLeft => "antenna_left".to_string(),
    }
}

/// The heading a joint's measured angle is written under.
///
/// Spelled here and nowhere else: the writer builds the header from it and
/// [`metrics`] finds its columns by it, so the file format is one string rather
/// than an agreement between two files that a rename would break silently.
pub(crate) fn present_heading(joint: JointId) -> String {
    format!("{}_present_rad", column(joint))
}

/// The heading a joint's commanded angle is written under.
pub(crate) fn goal_heading(joint: JointId) -> String {
    format!("{}_goal_rad", column(joint))
}

/// The header row, without its newline.
///
/// `run` distinguishes the moves appended to one file, including moves from
/// separate invocations of the bench; `phase` says whether the period was still
/// commanding or was one of those spent waiting for the machine to arrive,
/// which is the boundary the whole settle measurement is about.
fn header() -> String {
    let mut row = String::from("run,tick,t_s,phase");
    for joint in JointId::ALL {
        row.push_str(&format!(",{}", present_heading(joint)));
    }
    for joint in JointId::ALL {
        row.push_str(&format!(",{}", goal_heading(joint)));
    }
    row
}

/// Nine angles as nine cells, in bus order.
fn angles(row: &mut String, joints: &JointVector) {
    for (_, angle) in joints.joints() {
        row.push_str(&format!(",{angle:.6}"));
    }
}

/// Nine empty cells: a period whose grouped read fell short measured nothing,
/// and a zero there would read as nine joints at the origin.
fn blanks(row: &mut String) {
    for _ in 0..JointId::COUNT {
        row.push(',');
    }
}

/// The nine goals, with a released servo's cell left empty.
///
/// A servo taken out of service is torqued off and never written again, so it
/// is holding no goal — the same absence a blind read is, and rendered the same
/// way. The number that would otherwise stand there is the last goal it was
/// sent before the mask, repeated for every remaining period of the run, and a
/// reader differencing it against the measured column would see a command error
/// that was never on the wire.
fn goals(row: &mut String, joints: &JointVector, released: JointSet) {
    for (joint, angle) in joints.joints() {
        if released.contains(joint) {
            row.push(',');
        } else {
            row.push_str(&format!(",{angle:.6}"));
        }
    }
}

/// One sample as its row, without its newline.
fn row(run: u64, sample: &TickSample) -> String {
    let mut row = format!(
        "{run},{tick},{t:.6},{phase}",
        tick = sample.tick,
        t = sample.at.as_secs_f64(),
        phase = if sample.settling {
            "settling"
        } else {
            "commanding"
        },
    );
    match &sample.present {
        Some(present) => angles(&mut row, present),
        None => blanks(&mut row),
    }
    goals(&mut row, &sample.goal, sample.released);
    row
}

/// Render `samples` as CSV rows onto `out`, with a header first if `header` is
/// set.
///
/// Buffered by the caller and flushed by it: this writes rows and nothing else,
/// which is what lets a run of thousands of periods cost one syscall.
pub fn write_csv(
    out: &mut dyn Write,
    run: u64,
    samples: &[TickSample],
    with_header: bool,
) -> io::Result<()> {
    if with_header {
        writeln!(out, "{}", header())?;
    }
    for sample in samples {
        writeln!(out, "{}", row(run, sample))?;
    }
    Ok(())
}

/// What an append did to the file: which run the rows went in under, and
/// whether the rows already there ended mid-row.
pub struct Append {
    /// The number this run was written under.
    pub run: u64,
    /// The file's last row was incomplete — an earlier append cut short — and
    /// was terminated before these rows went in. The damaged row is left as it
    /// is; what this says is that the file was already not what it claims to
    /// be, so the operator hears it rather than reading a short row later and
    /// wondering.
    pub mended: bool,
}

/// Append one run's samples to the file at `path`, writing the header if the
/// file is new or empty, and answering with the number the run was written
/// under.
///
/// Appended rather than replaced so a session's moves land in one file in the
/// order they were commanded, and the `run` column is what separates them. The
/// number continues from whatever the file already holds rather than counting
/// from the caller: a session is a sequence of bench invocations sharing one
/// file, each of which starts its own engagement, so a per-caller counter would
/// write `0` over every move in the file and leave a reader grouping by `run`
/// with one merged series.
///
/// A file whose last row is incomplete gets that row terminated first, so this
/// run's first period is a row of its own rather than the second half of a row
/// that was cut short.
///
/// The write is buffered and flushed once, at the end. A trace is diagnostic
/// output: nothing about a control period may wait on a filesystem, and the
/// destination is expected to be a memory filesystem in any case.
pub fn append_csv(path: &Path, samples: &[TickSample]) -> io::Result<Append> {
    let mut file = OpenOptions::new()
        .create(true)
        .read(true)
        .append(true)
        .open(path)?;
    let len = file.metadata()?.len();
    let tail = tail(&mut file, len)?;
    let mended = !tail.is_empty() && !tail.ends_with('\n');
    let run = next_run(&tail, len > TAIL_BYTES);
    let mut out = BufWriter::new(file);
    if mended {
        writeln!(out)?;
    }
    write_csv(&mut out, run, samples, len == 0)?;
    out.flush()?;
    Ok(Append { run, mended })
}

/// The last [`TAIL_BYTES`] of the file, as text.
fn tail(file: &mut File, len: u64) -> io::Result<String> {
    if len == 0 {
        return Ok(String::new());
    }
    file.seek(SeekFrom::Start(len.saturating_sub(TAIL_BYTES)))?;
    // Bytes, then lossy: a tail cut mid-character must not fail the trace over
    // an encoding.
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(String::from_utf8_lossy(&bytes).into_owned())
}

/// The number the next run written to this file takes: one past the highest any
/// complete row in `tail` carries, or zero when no row there carries one.
///
/// The file's own rows are the record of what has been written to it, which is
/// what makes this work across invocations — nothing is remembered in a
/// process. The highest rather than the last, and complete rows only, because
/// the two rows a tail can lie about are its first — cut by where the read
/// started, when `cut` says the tail is not the whole file — and its last, cut
/// by an append that was interrupted mid-row. Either can carry a fragment of a
/// number that still parses, and a number already in the file is the one thing
/// this must not answer: two physically distinct moves sharing a `run` merge
/// into one series for every reader that groups by it.
///
/// A tail with no parseable number at all is a file holding only its header, or
/// one an operator has edited; zero is the honest answer and the rows say what
/// they say.
fn next_run(tail: &str, cut: bool) -> u64 {
    let mut lines: Vec<&str> = tail.lines().collect();
    if cut && !lines.is_empty() {
        lines.remove(0);
    }
    if !tail.ends_with('\n') && !lines.is_empty() {
        lines.pop();
    }
    let highest = lines
        .iter()
        .filter_map(|line| line.split(',').next())
        .filter_map(|cell| cell.trim().parse::<u64>().ok())
        .max();
    highest.map_or(0, |run| run.saturating_add(1))
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use super::*;

    /// A sample with `present` at `angle` on every joint and the goal at zero.
    fn sample(tick: u64, angle: Option<f64>, settling: bool) -> TickSample {
        TickSample {
            tick,
            at: Duration::from_millis(20 * tick),
            present: angle.map(|angle| JointVector {
                body_yaw: angle,
                legs: [angle; 6],
                antennas: [angle; 2],
            }),
            goal: JointVector::default(),
            released: JointSet::EMPTY,
            settling,
        }
    }

    /// The header names every joint twice — measured and commanded — and the
    /// rows carry exactly that many cells, so a reader splitting on commas can
    /// index a column by its heading.
    #[test]
    fn every_row_has_a_cell_for_every_column() {
        let mut out = Vec::new();
        let samples = [sample(0, Some(0.5), false), sample(1, None, true)];
        write_csv(&mut out, 3, &samples, true).expect("a vector takes writes");
        let text = String::from_utf8(out).expect("the rows are text");

        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        let columns = lines[0].split(',').count();
        assert_eq!(columns, 4 + 2 * JointId::COUNT);
        for line in &lines[1..] {
            assert_eq!(line.split(',').count(), columns, "{line}");
        }
    }

    /// A period with no read leaves its measured cells empty rather than
    /// standing in a number: nine zeros would read as nine joints at the
    /// origin, which is a pose and not an absence.
    #[test]
    fn a_blind_period_measures_nothing_and_says_so() {
        let mut out = Vec::new();
        write_csv(&mut out, 1, &[sample(7, None, true)], false).expect("a vector takes writes");
        let text = String::from_utf8(out).expect("the row is text");

        let cells: Vec<&str> = text.trim_end().split(',').collect();
        assert_eq!(cells[0], "1", "the run column");
        assert_eq!(cells[1], "7", "the period");
        assert_eq!(cells[3], "settling");
        for cell in &cells[4..4 + JointId::COUNT] {
            assert!(cell.is_empty(), "{text}");
        }
        for cell in &cells[4 + JointId::COUNT..] {
            assert_eq!(*cell, "0.000000", "{text}");
        }
    }

    /// A servo that has been released is commanded nothing, so its goal cell is
    /// empty while its measured cell keeps recording.
    ///
    /// This is the trace's job at exactly the moment it is the diagnostic of
    /// record: a degraded pair drifts limp for the rest of the run, and the
    /// last goal it was sent before the mask, held in the cell and repeated, is
    /// a command that was never on the wire.
    #[test]
    fn a_released_servo_is_commanded_nothing_and_keeps_measuring() {
        let mut released = JointSet::EMPTY;
        released.insert(JointId::AntennaRight);
        released.insert(JointId::AntennaLeft);
        let angles = JointVector {
            body_yaw: 0.5,
            legs: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            antennas: [7.0, 8.0],
        };
        let mut out = Vec::new();
        let sample = TickSample {
            tick: 4,
            at: Duration::from_millis(80),
            present: Some(angles),
            goal: angles,
            released,
            settling: false,
        };
        write_csv(&mut out, 0, &[sample], false).expect("a vector takes writes");
        let text = String::from_utf8(out).expect("the row is text");

        let cells: Vec<&str> = text.trim_end().split(',').collect();
        for (index, (joint, angle)) in angles.joints().into_iter().enumerate() {
            let measured = cells[4 + index];
            let commanded = cells[4 + JointId::COUNT + index];
            assert_eq!(measured, format!("{angle:.6}"), "{joint} measured: {text}");
            if released.contains(joint) {
                assert!(commanded.is_empty(), "{joint} commanded: {text}");
            } else {
                assert_eq!(commanded, format!("{angle:.6}"), "{joint} commanded");
            }
        }
    }

    /// Every measured column stands over the joint the header names, and every
    /// commanded column over the same joint again.
    ///
    /// The headings are what a reader indexes a series by, and nothing else
    /// would notice a column named for one joint carrying another's angle: the
    /// row would still have nine cells, the file would still parse, and a guard
    /// calibrated from it would be calibrated against the wrong joint.
    #[test]
    fn every_column_stands_over_the_joint_it_names() {
        let mut out = Vec::new();
        let angles = JointVector {
            body_yaw: 0.5,
            legs: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            antennas: [7.0, 8.0],
        };
        let written = TickSample {
            tick: 0,
            at: Duration::ZERO,
            present: Some(angles),
            goal: angles,
            released: JointSet::EMPTY,
            settling: false,
        };
        write_csv(&mut out, 0, &[written], true).expect("a vector takes writes");
        let text = String::from_utf8(out).expect("the rows are text");

        let lines: Vec<&str> = text.lines().collect();
        let headings: Vec<&str> = lines[0].split(',').collect();
        let cells: Vec<&str> = lines[1].split(',').collect();
        for (index, (joint, angle)) in angles.joints().into_iter().enumerate() {
            let key = column(joint);
            assert_eq!(headings[4 + index], format!("{key}_present_rad"));
            assert_eq!(
                headings[4 + JointId::COUNT + index],
                format!("{key}_goal_rad")
            );
            // The angle under the heading is that joint's, which is what makes
            // the heading a key rather than a label.
            assert_eq!(cells[4 + index], format!("{angle:.6}"), "{key} measured");
            assert_eq!(
                cells[4 + JointId::COUNT + index],
                format!("{angle:.6}"),
                "{key} commanded"
            );
        }
    }

    /// A second run appends to the first and does not repeat the header: one
    /// file per session, one row per period, the `run` column telling the moves
    /// apart.
    #[test]
    fn a_session_appends_its_runs_to_one_file() {
        let path = crate::testutil::scratch_path("trace.csv");

        let first =
            append_csv(&path, &[sample(0, Some(0.1), false)]).expect("the first run writes");
        let second =
            append_csv(&path, &[sample(0, Some(0.2), false)]).expect("the second run appends");
        let text = std::fs::read_to_string(&path).expect("the file is there");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!((first.run, second.run), (0, 1));
        assert!(!first.mended && !second.mended, "nothing was cut short");
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3, "{text}");
        assert_eq!(lines[0], header());
        assert!(lines[1].starts_with("0,"), "{text}");
        assert!(lines[2].starts_with("1,"), "{text}");
    }

    /// The run numbers come from the file rather than from the writer, so moves
    /// written by separate invocations of the bench are still told apart.
    ///
    /// This is the primary workflow — `up --trace f.csv` then `stow --trace
    /// f.csv` — and each of those invocations remembers nothing of the last.
    #[test]
    fn a_run_written_by_a_later_invocation_does_not_reuse_a_number() {
        let path = crate::testutil::scratch_path("invocations.csv");

        // Three separate calls with nothing carried between them, which is what
        // three bench commands appending to one file amount to.
        let mut written = Vec::new();
        for period in 0..3u64 {
            written.push(
                append_csv(&path, &[sample(period, Some(0.1), false)])
                    .expect("the run appends")
                    .run,
            );
        }
        let text = std::fs::read_to_string(&path).expect("the file is there");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!(written, vec![0, 1, 2]);
        let runs: Vec<&str> = text
            .lines()
            .skip(1)
            .map(|line| line.split(',').next().expect("a row has a first cell"))
            .collect();
        assert_eq!(runs, vec!["0", "1", "2"], "{text}");
    }

    /// A file holding only its header has no run in it yet, so the next one is
    /// the first.
    #[test]
    fn a_file_with_no_rows_starts_its_numbering_at_zero() {
        let path = crate::testutil::scratch_path("header-only.csv");
        std::fs::write(&path, format!("{}\n", header())).expect("the header writes");

        let run = append_csv(&path, &[sample(0, Some(0.1), false)]).expect("the run appends");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!(run.run, 0);
        assert!(!run.mended, "a header is a complete line");
    }

    /// A file long enough that the tail read starts part way into a row still
    /// numbers the next run from what the file holds.
    ///
    /// The whole reason the tail is read rather than the file is a session of a
    /// few dozen runs, and every case above fits in one read from byte zero. A
    /// seek off by a row — or a leading fragment that happens to parse — would
    /// renumber runs in exactly the long file an operator is grouping by `run`.
    #[test]
    fn a_file_past_the_tail_read_numbers_from_the_rows_it_holds() {
        let path = crate::testutil::scratch_path("long-session.csv");

        // Rows are a few hundred bytes, so this is comfortably past the tail
        // window and leaves the read starting mid-row.
        let long: Vec<TickSample> = (0..40).map(|tick| sample(tick, Some(0.1), false)).collect();
        let first = append_csv(&path, &long).expect("the long run writes");
        let len = std::fs::metadata(&path).expect("the file is there").len();
        let second = append_csv(&path, &[sample(0, Some(0.2), false)]).expect("the run appends");
        let text = std::fs::read_to_string(&path).expect("the file is there");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert!(
            len > TAIL_BYTES,
            "the tail read starts past byte zero: {len}"
        );
        // And it starts inside a row rather than on one, which is the case the
        // leading line is discarded for: a fragment of a row whose first cell
        // is some other column's number.
        let mut at = 0u64;
        let starts: Vec<u64> = text
            .lines()
            .map(|line| {
                let start = at;
                at += line.len() as u64 + 1;
                start
            })
            .collect();
        assert!(
            !starts.contains(&(len - TAIL_BYTES)),
            "the tail is cut mid-row: {len}"
        );
        assert_eq!((first.run, second.run), (0, 1));
        let last = text.lines().last().expect("the file has rows");
        assert!(last.starts_with("1,"), "{last}");
    }

    /// An append interrupted mid-row does not cost the next run its own number,
    /// and does not leave the next row welded onto the damaged one.
    ///
    /// A run is flushed to the file as it goes, so an interrupt can land inside
    /// a row: its `run` cell is then a fragment that still parses as a number.
    /// Reading only the last line would take that fragment for the file's
    /// numbering and hand the next run a number already in use — two distinct
    /// moves merged into one series for anything that groups by `run`.
    #[test]
    fn a_row_cut_short_by_an_interrupted_append_collides_with_nothing() {
        let path = crate::testutil::scratch_path("interrupted.csv");

        let mut written = format!("{}\n", header());
        for run in 0..12u64 {
            written.push_str(&format!("{}\n", row(run, &sample(0, Some(0.1), false))));
        }
        // Run 12's first row, cut mid-way: "12" survives as "1".
        let cut = row(12, &sample(0, Some(0.1), false));
        written.push_str(&cut[..1]);
        std::fs::write(&path, &written).expect("the damaged file writes");

        let appended = append_csv(&path, &[sample(0, Some(0.2), false)]).expect("the run appends");
        let text = std::fs::read_to_string(&path).expect("the file is there");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!(
            appended.run, 12,
            "past every complete row, not past the fragment: {text}"
        );
        assert!(appended.mended, "the damaged row was said, not swallowed");
        let last = text.lines().last().expect("the file has rows");
        assert!(
            last.starts_with("12,"),
            "the new row stands on its own: {last}"
        );
    }

    /// A file whose rows carry no number the reader can use starts again at
    /// zero rather than guessing.
    #[test]
    fn a_file_with_no_parseable_rows_starts_its_numbering_at_zero() {
        let path = crate::testutil::scratch_path("hand-edited.csv");
        std::fs::write(&path, format!("{}\nnot,a,row,at,all\n", header()))
            .expect("the file writes");

        let appended = append_csv(&path, &[sample(0, Some(0.1), false)]).expect("the run appends");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!(appended.run, 0);
    }

    /// A last line the reader cannot number does not send the count back to
    /// zero over rows that are already numbered.
    ///
    /// Zero is the honest answer for a file with no runs in it. In a file that
    /// holds run 0 it is a collision: two physically distinct moves under one
    /// number, merged into one series by anything that groups on it.
    #[test]
    fn a_line_the_reader_cannot_number_does_not_reset_the_count() {
        let path = crate::testutil::scratch_path("annotated.csv");
        let mut written = format!("{}\n", header());
        for run in 0..3u64 {
            written.push_str(&format!("{}\n", row(run, &sample(0, Some(0.1), false))));
        }
        written.push_str("# the antenna clipped here\n");
        std::fs::write(&path, &written).expect("the annotated file writes");

        let appended = append_csv(&path, &[sample(0, Some(0.2), false)]).expect("the run appends");
        let text = std::fs::read_to_string(&path).expect("the file is there");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        assert_eq!(appended.run, 3, "past the rows that are numbered: {text}");
        assert!(!appended.mended, "every line was whole");
    }
}
