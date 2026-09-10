//! A window of one run's driver samples, written as a replay trace.
//!
//! `bazel run //cogs:trace_export -- <log-dir> <from-ns> <to-ns> <out.csv>`.
//! The two instants are nominal times on the driver's own grid, which is what
//! every other tool over these logs reports and the two the library tour's
//! report prints per window.
//!
//! What it is for is cutting a fixture. The replay suite in `reachy-motion`
//! judges recorded runs offline through the same functions the decision tick
//! runs live, and what it reads is a CSV of periods; a hardware run arrives as
//! a Clockwork log instead. This turns a stretch of one into the other, so an
//! expensive hardware round trip leaves a checked-in regression asset behind
//! rather than a figure somebody wrote down.
//!
//! A pure copy, not a judgment: no verdict, no threshold, no interpolation. A
//! period is copied out whole -- the reading if the sample carried one, the
//! setpoint the driver was holding, blank where it held none.
//!
//! Sans-log in the same sense as the rest of the analyzers: the writing is a
//! pure function over the samples, and `main` is what binds a log to it.

#![forbid(unsafe_code)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, read_with, typed};
use motion_channels::POSE_CHANNEL;
use pose_reading::{Grid, commanded_rows, present_rows};
use reachy_driver::NOMINAL_CYCLE_NS;
use reachy_motion::joints::ROWS;
use reachy_motion::trace::{goal_column, present_column};

/// The sample stream, and nothing else.
///
/// A cut is a copy of what the driver recorded. The schedule stream says which
/// instants are worth cutting, but reading it here would make this tool an
/// opinion about which window a fixture is; the instants are the caller's.
#[derive(Default)]
struct Run {
    /// The driver's heartbeat: one per cycle.
    samples: Vec<Logged<PoseSampleWire>>,
    /// Every channel the log carries and how many messages each held.
    census: Census,
    /// Anything that went wrong reading the log itself.
    complaints: Complaints,
}

impl Streams for Run {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }

    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

/// The one channel this tool reads.
const CHANNELS: [Bound<Run>; 1] = [Bound {
    name: POSE_CHANNEL,
    check: binding::<PoseSampleWire>,
    route: |run, message| typed(message, &mut run.samples, &mut run.complaints),
}];

impl Run {
    /// Read the log under `dir`.
    ///
    /// # Errors
    ///
    /// Whatever the shared pass refuses about the log as a whole.
    fn read(dir: &Path) -> Result<Self, clockwork_logs::LogError> {
        read_with(dir, &CHANNELS)
    }
}

/// The trace CSV's header, and the order every row's cells are written in.
///
/// The nine readings in bus order, then the nine setpoints in bus order, which
/// is the shape the replay suite's parser resolves by name. Both halves of the
/// format's vocabulary come from the motion crate: the column names are
/// `trace::present_column` and `trace::goal_column`, the same two functions the
/// parser resolves the header with, and the order is `joints::ROWS` -- so the
/// tool that writes a fixture and the suite that reads one cannot disagree
/// about which crank a column belongs to.
#[must_use]
pub fn header() -> String {
    let mut header = String::from("run,tick,t_s,phase");
    for joint in ROWS {
        let _ = write!(header, ",{}", present_column(joint));
    }
    for joint in ROWS {
        let _ = write!(header, ",{}", goal_column(joint));
    }
    header
}

/// The samples between two nominal instants, inclusive of both, as a trace.
///
/// One row per sample, in grid order. The `tick` column counts periods from the
/// first sample in the window rather than from the run's own start, because a
/// trace is replayed on its own and its first period is period zero; `t_s`
/// counts seconds from the same instant. Every row's phase is `commanding`: a
/// driver sample says what was held, not whether the mover thought it had
/// finished, and a fixture cut from a moving machine is a fixture of a machine
/// being commanded.
///
/// A period whose grouped read fell short writes nine blank reading cells, and
/// a period the driver held no setpoint on writes nine blank goal cells -- both
/// are what the parser reads as absent, and both are facts about the run that a
/// zero would misreport.
///
/// # Errors
///
/// A window holding no sample. Everything else about the stream is the log's
/// own: samples arriving out of order are sorted, and a sample carrying a
/// reading that does not validate is written blank the way the analyzers read
/// it.
pub fn trace(
    samples: &[Logged<PoseSampleWire>],
    from_ns: i64,
    to_ns: i64,
) -> Result<String, String> {
    let mut cut: Vec<&Logged<PoseSampleWire>> = samples
        .iter()
        .filter(|sample| {
            let at = sample.message.nominal_time().as_nanos();
            (from_ns..=to_ns).contains(&at)
        })
        .collect();
    cut.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
    let first = cut
        .first()
        .ok_or_else(|| format!("no sample sits between {from_ns} and {to_ns}"))?;
    let grid = Grid {
        origin_ns: first.message.nominal_time().as_nanos(),
        period_ns: NOMINAL_CYCLE_NS,
    };

    let mut out = header();
    out.push('\n');
    for sample in cut {
        let at_ns = sample.message.nominal_time().as_nanos();
        let (tick, _) = grid.at(at_ns);
        let secs = (at_ns - grid.origin_ns) as f64 / 1e9;
        // The `run` column is zero throughout: a cut is one stretch of one
        // run.
        let _ = write!(out, "0,{tick},{secs:.6},commanding");
        cells(&mut out, present_rows(&sample.message).as_ref());
        cells(&mut out, commanded_rows(&sample.message).as_ref());
        out.push('\n');
    }
    Ok(out)
}

/// Nine angle cells, or nine blank ones where the sample carried none.
fn cells(out: &mut String, angles: Option<&[f64; ROWS.len()]>) {
    for row in 0..ROWS.len() {
        match angles {
            Some(angles) => {
                let _ = write!(out, ",{:.6}", angles[row]);
            }
            None => out.push(','),
        }
    }
}

fn main() -> ExitCode {
    const USAGE: &str = "usage: trace_export <log-dir> <from-ns> <to-ns> <out.csv>";
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [log_dir, from, to, out_path] = args.as_slice() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let (Ok(from_ns), Ok(to_ns)) = (from.parse::<i64>(), to.parse::<i64>()) else {
        eprintln!("{USAGE}");
        eprintln!("the window's two ends are nominal instants in nanoseconds");
        return ExitCode::FAILURE;
    };
    let run = match Run::read(&PathBuf::from(log_dir)) {
        Ok(run) => run,
        Err(err) => {
            eprintln!("reading the log under {log_dir}: {err}");
            return ExitCode::FAILURE;
        }
    };
    for complaint in &run.complaints {
        eprintln!("{complaint}");
    }
    if !run.complaints.is_empty() {
        // A message the reader could not decode is a period missing from the
        // cut, and a fixture with a hole in it is worse than no fixture.
        return ExitCode::FAILURE;
    }
    let text = match trace(&run.samples, from_ns, to_ns) {
        Ok(text) => text,
        Err(err) => {
            eprintln!("cutting {log_dir}: {err}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(err) = std::fs::write(out_path, &text) {
        eprintln!("writing {out_path}: {err}");
        return ExitCode::FAILURE;
    }
    // One line per period plus the header, so the operator who cut the window
    // can check it against the span the report printed.
    println!(
        "{out_path}: {} period(s) from {from_ns} to {to_ns}",
        text.lines().count() - 1
    );
    ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    //! What the cut is, over a crafted sample stream.

    use super::{Logged, PoseSampleWire, header, trace};

    use clockwork_rs::SyncTime;
    use reachy_driver::NOMINAL_CYCLE_NS;
    use reachy_motion::joints::{ROW_COUNT, write_rows};

    /// The instant the crafted run's first period sits on.
    const ORIGIN_NS: i64 = 5_000_000_000;

    /// One sample on the grid: cycle `cycle`, with the reading and the setpoint
    /// each present or absent.
    fn sample(
        cycle: i64,
        present: Option<[f64; ROW_COUNT]>,
        commanded: Option<[f64; ROW_COUNT]>,
    ) -> Logged<PoseSampleWire> {
        let at_ns = ORIGIN_NS + cycle * NOMINAL_CYCLE_NS;
        let mut message = PoseSampleWire::new();
        {
            let read = message.clear_valid();
            read.nominal_time = SyncTime::from_nanos(at_ns);
            read.sample_time = read.nominal_time;
            read.present_valid = present.is_some().into();
            read.commanded_valid = commanded.is_some().into();
            write_rows(&mut read.present, &present.unwrap_or([0.0; ROW_COUNT]));
            write_rows(&mut read.commanded, &commanded.unwrap_or([0.0; ROW_COUNT]));
        }
        Logged {
            at_ns,
            sequence_number: u32::try_from(cycle).expect("a few periods"),
            message,
        }
    }

    /// The nine angles of period `cycle`, distinct per row and per period so a
    /// column swap is visible.
    fn angles(cycle: i64, offset: f64) -> [f64; ROW_COUNT] {
        let mut rows = [0.0; ROW_COUNT];
        for (row, angle) in rows.iter_mut().enumerate() {
            *angle = offset + cycle as f64 + row as f64 / 100.0;
        }
        rows
    }

    /// The header names both halves of every bus row, and names them in the
    /// order the cells are written.
    #[test]
    fn the_header_names_a_reading_and_a_setpoint_for_every_row() {
        let header = header();
        let columns: Vec<&str> = header.split(',').collect();
        assert_eq!(columns.len(), 4 + 2 * ROW_COUNT);
        assert_eq!(&columns[..4], ["run", "tick", "t_s", "phase"]);
        assert_eq!(columns[4], "body_yaw_present_rad");
        assert_eq!(columns[5], "leg1_present_rad");
        assert_eq!(columns[10], "leg6_present_rad");
        assert_eq!(columns[11], "antenna_right_present_rad");
        assert_eq!(columns[12], "antenna_left_present_rad");
        assert_eq!(columns[13], "body_yaw_goal_rad");
        assert_eq!(columns[21], "antenna_left_goal_rad");
    }

    /// A window is cut inclusive of both instants, counted from its own first
    /// period, and carries every angle the samples did.
    #[test]
    fn a_window_is_cut_from_its_own_first_period() {
        let samples: Vec<_> = (0..10)
            .map(|cycle| sample(cycle, Some(angles(cycle, 0.0)), Some(angles(cycle, 100.0))))
            .collect();
        let text = trace(
            &samples,
            ORIGIN_NS + 3 * NOMINAL_CYCLE_NS,
            ORIGIN_NS + 5 * NOMINAL_CYCLE_NS,
        )
        .expect("the window holds samples");

        let rows: Vec<&str> = text.lines().collect();
        assert_eq!(rows.len(), 4, "a header and three periods: {text}");
        assert_eq!(rows[0], header());
        let first: Vec<&str> = rows[1].split(',').collect();
        assert_eq!(&first[..4], ["0", "0", "0.000000", "commanding"]);
        assert_eq!(first[4], "3.000000", "the reading of period three");
        assert_eq!(first[13], "103.000000", "and the setpoint it was held to");
        let last: Vec<&str> = rows[3].split(',').collect();
        assert_eq!(
            &last[..4],
            ["0", "2", "0.040000", "commanding"],
            "the last period of the window is two periods in"
        );
        assert_eq!(last[12], "5.080000", "the left antenna's own reading");
    }

    /// A period the driver read nothing on, and a period it held nothing on,
    /// write blanks rather than zeros -- which is what the replay parser reads
    /// as absent.
    #[test]
    fn a_period_missing_a_half_writes_that_half_blank() {
        let samples = vec![
            sample(0, None, Some(angles(0, 100.0))),
            sample(1, Some(angles(1, 0.0)), None),
        ];
        let text = trace(&samples, ORIGIN_NS, ORIGIN_NS + NOMINAL_CYCLE_NS)
            .expect("the window holds samples");
        let rows: Vec<&str> = text.lines().collect();

        let read_less: Vec<&str> = rows[1].split(',').collect();
        assert!(
            read_less[4..4 + ROW_COUNT]
                .iter()
                .all(|cell| cell.is_empty()),
            "the reading is blank: {}",
            rows[1]
        );
        assert_eq!(
            read_less[4 + ROW_COUNT],
            "100.000000",
            "the setpoint is not"
        );

        let held_nothing: Vec<&str> = rows[2].split(',').collect();
        assert_eq!(held_nothing[4], "1.000000", "the reading is there");
        assert!(
            held_nothing[4 + ROW_COUNT..]
                .iter()
                .all(|cell| cell.is_empty()),
            "and no setpoint is claimed: {}",
            rows[2]
        );
    }

    /// A cycle the log carries no sample for leaves a hole in the trace: the
    /// periods around it keep the slots they sat on, and their seconds agree
    /// with those slots.
    ///
    /// The one input shape the replay side treats specially, and the shape a
    /// real log has whenever the driver dropped a cycle. Both timing columns are
    /// derived from the same nanoseconds by different arithmetic -- the slot
    /// off the grid, the seconds off the origin -- so a hole is where they could
    /// disagree, and a fixture whose periods were renumbered across one would
    /// read as a residual nobody could explain against the report the window
    /// was cut from.
    #[test]
    fn a_cycle_the_log_missed_leaves_a_hole_in_the_trace() {
        let samples: Vec<_> = [0, 1, 3, 4]
            .into_iter()
            .map(|cycle| sample(cycle, Some(angles(cycle, 0.0)), Some(angles(cycle, 100.0))))
            .collect();
        let text = trace(&samples, ORIGIN_NS, ORIGIN_NS + 4 * NOMINAL_CYCLE_NS)
            .expect("the window holds samples");

        let rows: Vec<&str> = text.lines().skip(1).collect();
        assert_eq!(rows.len(), 4, "four periods and no filling: {text}");
        for (row, cycle) in rows.iter().zip([0, 1, 3, 4]) {
            let cells: Vec<&str> = row.split(',').collect();
            assert_eq!(cells[1], cycle.to_string(), "the slot it sat on: {row}");
            let secs = cycle as f64 * NOMINAL_CYCLE_NS as f64 / 1e9;
            assert_eq!(
                cells[2],
                format!("{secs:.6}"),
                "and the same instant: {row}"
            );
            assert_eq!(
                cells[4],
                format!("{:.6}", cycle as f64),
                "carrying that period's own reading: {row}"
            );
        }
    }

    /// The cut is over the grid and not over the log's order: a stream that
    /// arrived out of turn writes the same trace.
    #[test]
    fn samples_out_of_order_cut_the_same_trace() {
        let mut samples: Vec<_> = (0..6)
            .map(|cycle| sample(cycle, Some(angles(cycle, 0.0)), Some(angles(cycle, 100.0))))
            .collect();
        let ordered = trace(&samples, ORIGIN_NS, ORIGIN_NS + 5 * NOMINAL_CYCLE_NS)
            .expect("the window holds samples");
        samples.swap(1, 4);
        let shuffled = trace(&samples, ORIGIN_NS, ORIGIN_NS + 5 * NOMINAL_CYCLE_NS)
            .expect("the window holds samples");
        assert_eq!(ordered, shuffled);
    }

    /// A window nothing sits in is refused rather than written empty: a
    /// zero-period fixture parses and measures nothing.
    #[test]
    fn a_window_with_no_sample_in_it_is_refused() {
        let samples = vec![sample(0, Some(angles(0, 0.0)), Some(angles(0, 100.0)))];
        let error = trace(&samples, ORIGIN_NS + 1, ORIGIN_NS + 2).expect_err("nothing to cut");
        assert!(error.contains("no sample"), "{error}");
    }
}
