//! Reading a driver's sample stream, for whatever analyzer is reading it.
//!
//! Two tools judge two different runs off the same log: the wake gesture's
//! report and the library tour's. What they conclude is their own, but the
//! vocabulary underneath — what a sample's nine angles are, how far the machine
//! ran behind its goals, what grid the samples sit on, and which slots the
//! driver already said it missed — is one subject and belongs in one place.
//!
//! The reason it is shared rather than copied is what an operator does with the
//! output: a lag figure from a gesture run and a lag figure from a tour run are
//! compared against each other and against the recorded gestures, so the two
//! must be the same measurement said the same way. A second copy diverges on
//! the first change nobody made twice, and the divergence is invisible until
//! somebody reads two runs side by side.
//!
//! Sans-log: everything here takes the messages it needs and a report to write
//! into. Which channels those came off, and which of them a tool binds, is the
//! tool's.

#![forbid(unsafe_code)]

use std::collections::BTreeSet;

use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire};
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use log_read::Logged;
use reachy_motion::joints::{JointGroup, ROWS, group_of, row, rows_of};
use reachy_motion::plant::{MAX_GAP_PERIODS, PlantModel, Predicted, RESPONSE_DEAD_SAMPLES};
use reachy_motion::tick::{
    RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_ANTENNA_RESIDUAL_RAD,
    RECORDED_WORST_HEAD_LAG_RAD, RECORDED_WORST_HEAD_RESIDUAL_RAD, TrackingFaultConfig,
};
use run_report::Report;

/// The nine angles a sample was read at, or nothing where it carried no
/// reading.
#[must_use]
pub fn present_rows(sample: &PoseSampleWire) -> Option<[f64; ROWS.len()]> {
    if !sample.present_valid() {
        return None;
    }
    sample.present().validate().ok().map(rows_of)
}

/// The nine angles a sample was holding, or nothing where it held none.
#[must_use]
pub fn commanded_rows(sample: &PoseSampleWire) -> Option<[f64; ROWS.len()]> {
    if !sample.commanded_valid() {
        return None;
    }
    sample.commanded().validate().ok().map(rows_of)
}

/// How far the machine ran behind what it was told to hold, over a whole run.
///
/// Off the samples alone: each carries the setpoint the driver is holding
/// beside the position it read, so the lag needs no join against the goal
/// stream. Two figures, head and antennas, because those are the two the
/// recorded hardware gestures pinned — and both of those numbers are printed
/// beside the measurement, so a run can be read against the recordings.
///
/// What this is a figure about is how fast the content is against the servos'
/// own profile, not how healthy the machine is: every joint on this machine
/// runs a velocity-capped generator, so content asking for more than the cap
/// leaves a healthy joint far behind its goal and says nothing wrong. Health is
/// [`residuals`], which is what the detector screens on.
pub fn lags(samples: &[Logged<PoseSampleWire>], report: &mut Report) {
    let mut head = 0_f64;
    let mut antenna = 0_f64;
    let mut compared = 0_usize;
    for sample in samples {
        let (Some(present), Some(commanded)) = (
            present_rows(&sample.message),
            commanded_rows(&sample.message),
        ) else {
            continue;
        };
        compared += 1;
        for joint in ROWS {
            let Some(index) = row(joint) else { continue };
            let lag = (commanded[index] - present[index]).abs();
            match group_of(joint) {
                Some(JointGroup::Antennas) => antenna = antenna.max(lag),
                Some(_) => head = head.max(lag),
                None => {}
            }
        }
    }
    // How many samples the two figures came off, because a zero lag and a
    // measurement nothing was compared on print the same otherwise -- and a run
    // in which the driver held nothing is the second one.
    report.note(format!(
        "{compared} of {} samples carried both a reading and a setpoint to compare",
        samples.len()
    ));
    report.note(format!(
        "worst head lag {head:.4} rad, distance behind the goal; the recorded healthy gesture \
         ran at {RECORDED_WORST_HEAD_LAG_RAD:.4} rad"
    ));
    report.note(format!(
        "worst antenna lag {antenna:.4} rad; the recorded fast sweep ran at \
         {RECORDED_WORST_ANTENNA_LAG_RAD:.4} rad"
    ));
}

/// The profile a log is to be judged under, read from the deployment's own
/// configuration file: acceleration first, then velocity, in register units.
///
/// A path argument rather than a constant, the way the names sidecar is. A run
/// recorded on a machine commissioned with one pair has to be judged under that
/// pair — the residual is the distance from a trajectory those two registers
/// define, so judging a log under any other pair measures a machine nobody ran.
///
/// The parse is a literal `key: value` scan over the lines that are not
/// comments: two scalars in a file this repo writes are not a reason to
/// implement protobuf text. Comments are skipped rather than scanned because
/// the file's own comment block discusses other pairs -- the bench's among them
/// -- and a scan that took the first match anywhere would read one of those as
/// the pair the machine was commissioned with.
///
/// # Errors
///
/// The reason the file could not be read, or the name it does not state.
pub fn read_profile(path: &str) -> Result<(u32, u32), String> {
    let text = std::fs::read_to_string(path).map_err(|error| format!("{path}: {error}"))?;
    let figure = |field: &str| -> Result<u32, String> {
        text.lines()
            .filter(|line| !line.trim_start().starts_with('#'))
            .filter_map(|line| line.split_once(':'))
            .find(|(name, _)| name.trim() == field)
            .ok_or_else(|| format!("{path} states no {field}"))?
            .1
            .trim()
            .parse()
            .map_err(|error| format!("{path}'s {field} is no register value: {error}"))
    };
    Ok((figure("profile_acceleration")?, figure("profile_velocity")?))
}

/// How far each joint stood from where its servo's own trajectory generator
/// had got to, sample by sample.
///
/// The offline half of the live comparison, stepped the way the tick steps it
/// so that a run is judged offline by the arithmetic that judged it live:
/// seeded from the first reading at rest, then one step-then-push per grid
/// period against the setpoint the driver was holding
/// [`RESPONSE_DEAD_SAMPLES`] periods earlier. A grid slot no sample attended is
/// stepped on the setpoint already held, because that is what the servos went
/// on chasing; a run of them longer than [`MAX_GAP_PERIODS`] re-seeds, and so
/// does a stretch where the driver held nothing at all, because past either the
/// generator's position is not something arithmetic knows.
///
/// One entry per sample that carried a reading, in nominal order: the instant
/// and the nine unsigned residuals. A joint *ahead* of its prediction is as far
/// off it as one behind, which is why they are unsigned — the question is
/// whether the reading and the model agree.
#[must_use]
pub fn residual_stream(
    samples: &[Logged<PoseSampleWire>],
    grid: Grid,
    plant: &PlantModel,
) -> Vec<(i64, [f64; ROWS.len()])> {
    // Nominal order is the model's own order, whatever order the log holds:
    // the prediction is a walk along the grid and a sample read out of turn
    // would step it backwards.
    let mut ordered: Vec<&Logged<PoseSampleWire>> = samples.iter().collect();
    ordered.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
    let mut predicted = [Predicted::default(); ROWS.len()];
    let mut seeded = [false; ROWS.len()];
    let mut ring: Vec<[f64; ROWS.len()]> = Vec::new();
    let mut previous: Option<i64> = None;
    let mut out = Vec::new();
    for sample in ordered {
        let nominal = sample.message.nominal_time().as_nanos();
        let (cycle, _) = grid.at(nominal);
        // How many periods this sample covers: its own, plus any slot no sample
        // attended. Floored at one, so a repeated or early instant is one
        // period and never none.
        let periods = previous.map_or(1, |before| (cycle - before).max(1));
        previous = Some(cycle);
        let held = commanded_rows(&sample.message);
        if periods > MAX_GAP_PERIODS as i64 || ring.len() < RESPONSE_DEAD_SAMPLES {
            seeded = [false; ROWS.len()];
            if periods > MAX_GAP_PERIODS as i64 {
                // The ring goes with the prediction over a gap that long, and
                // only over one: its setpoints are from before the gap, and a
                // prediction seeded from this sample has no business being
                // stepped toward one of them. A ring that is merely filling is
                // left to fill.
                ring.clear();
            }
        } else {
            for period in 0..periods {
                let target = ring.remove(0);
                for (index, state) in predicted.iter_mut().enumerate() {
                    if seeded[index] {
                        plant.step(state, target[index]);
                    }
                }
                if period + 1 < periods {
                    // The driver wrote nothing in a slot it missed, so the
                    // newest setpoint is pushed again: the servos chased what
                    // they were already holding.
                    let carried = *ring.last().unwrap_or(&target);
                    ring.push(carried);
                }
            }
        }
        match held {
            Some(setpoint) => {
                ring.push(setpoint);
                // The ring is exactly a dead time deep. A stretch the loop
                // above did not step through leaves it full, so the push above
                // drops the oldest rather than deepening it.
                while ring.len() > RESPONSE_DEAD_SAMPLES {
                    ring.remove(0);
                }
            }
            None => {
                ring.clear();
                seeded = [false; ROWS.len()];
            }
        }
        let Some(present) = present_rows(&sample.message) else {
            continue;
        };
        let mut residual = [0.0; ROWS.len()];
        for (index, state) in predicted.iter_mut().enumerate() {
            if !seeded[index] {
                state.position = present[index];
                state.velocity = 0.0;
                seeded[index] = true;
            }
            residual[index] = (present[index] - state.position).abs();
        }
        out.push((nominal, residual));
    }
    out
}

/// How far the machine stood from its own modelled trajectory, over a whole
/// run, printed against the screen the detector runs.
///
/// This is the figure a run is judged by. Unlike the lag above it, it does not
/// grow with the speed of the content: the model is the servo's own generator,
/// so a velocity-saturated clip and a slow gesture both sit near zero on a
/// healthy machine, and what puts a joint away from zero is the joint failing
/// to do what its own profile says it does.
///
/// The stream is the caller's, from [`residual_stream`], because the prediction
/// is a walk along the whole run and a caller that also slices it per window
/// would otherwise walk it twice -- and the summary and the slices under it must
/// be the same walk, or they are two answers to one question. `samples` is
/// carried only for the count of what the walk left out.
pub fn residuals(
    stream: &[(i64, [f64; ROWS.len()])],
    samples: &[Logged<PoseSampleWire>],
    plant: &PlantModel,
    report: &mut Report,
) {
    let mut head = 0_f64;
    let mut antenna = 0_f64;
    for (_, residual) in stream {
        for joint in ROWS {
            let Some(index) = row(joint) else { continue };
            match group_of(joint) {
                Some(JointGroup::Antennas) => antenna = antenna.max(residual[index]),
                Some(_) => head = head.max(residual[index]),
                None => {}
            }
        }
    }
    let threshold = TrackingFaultConfig::default().threshold_rad;
    report.note(format!(
        "{} of {} samples were judged against the modelled trajectory of a servo commissioned \
         at {:.6} rad/period and {:.6} rad/period²",
        stream.len(),
        samples.len(),
        plant.v_max,
        plant.a_max,
    ));
    report.note(format!(
        "worst head residual {head:.4} rad and worst antenna residual {antenna:.4} rad, against \
         a tracking screen at {threshold:.4} rad"
    ));
    report.note(format!(
        "the recorded library ran at {RECORDED_WORST_HEAD_RESIDUAL_RAD:.4} rad and \
         {RECORDED_WORST_ANTENNA_RESIDUAL_RAD:.4} rad, which is what the screen is sized over"
    ));
}

/// The decision tick raised nothing.
///
/// The count travels with the kind: on a tracking fault it is the window the
/// run was judged by, which is what says whether the figure was too short or
/// the run was never recognised as a reversal, and reading it out here saves a
/// dig through the log.
///
/// One wording for every run: a fault row that read differently depending on
/// which harness recorded it would be the same event with two names.
pub fn no_faults(faults: &[Logged<TickFaultWire>], report: &mut Report) {
    for fault in faults {
        report.fail(format!(
            "the decision tick raised {:?} at {}, count {}",
            fault.message.kind(),
            fault.message.time().as_nanos(),
            fault.message.count()
        ));
    }
}

/// The grid a run's samples sit on, derived from the samples themselves.
///
/// A hardware run starts at whatever top of a second the driver started at, so
/// nothing about the epoch can be assumed; what can be is the period, which is
/// the one number both hosts are built against. The origin is the first
/// sample's own nominal instant.
#[derive(Clone, Copy)]
pub struct Grid {
    /// The instant cycle zero of this run sits at.
    pub origin_ns: i64,
    /// How long one cycle is.
    pub period_ns: i64,
}

impl Grid {
    /// The cycle index of a nominal instant, and how far off the grid it sits.
    #[must_use]
    pub fn at(&self, nominal_ns: i64) -> (i64, i64) {
        let elapsed = nominal_ns - self.origin_ns;
        (
            elapsed.div_euclid(self.period_ns),
            elapsed.rem_euclid(self.period_ns),
        )
    }

    /// The same, with an instant within `jitter_ns` of a cycle counted as being
    /// on it.
    ///
    /// An instant that arrived late is over its own cycle's mark by the offset;
    /// one that arrived early is under the *next* cycle's, which the remainder
    /// reports as nearly a whole period. So both ends of the band are checked
    /// and the answer is the cycle the instant is nearest, with a zero offset
    /// when it is inside the band.
    #[must_use]
    pub fn within(&self, nominal_ns: i64, jitter_ns: i64) -> (i64, i64) {
        let (cycle, off) = self.at(nominal_ns);
        if off <= jitter_ns {
            (cycle, 0)
        } else if off >= self.period_ns - jitter_ns {
            (cycle + 1, 0)
        } else {
            (cycle, off)
        }
    }
}

/// The grid points the driver reported missing, and the reports themselves.
///
/// A skip report is published by the first cycle attended after the run of
/// missed slots, and it says how many they were, so the slots it accounts for
/// are the ones immediately before it. Which cycles those are is what lets a
/// hole in the sample stream be recognised as the same event rather than
/// counted a second time — which is the difference between a machine reading
/// and a harness defect.
pub struct Skips<'a> {
    /// Every skip report the run carried.
    pub events: Vec<&'a Logged<DriverEventWire>>,
    /// Every cycle a report accounts for.
    pub missed: BTreeSet<i64>,
}

impl<'a> Skips<'a> {
    /// The skips `events` reports, placed on `grid`.
    #[must_use]
    pub fn of(events: &'a [Logged<DriverEventWire>], grid: Grid, jitter_ns: i64) -> Self {
        let events: Vec<&Logged<DriverEventWire>> = events
            .iter()
            .filter(|event| event.message.kind() == EventKindWire::CYCLE_SKIPPED)
            .collect();
        let mut missed = BTreeSet::new();
        for event in &events {
            let (cycle, off) = grid.within(event.message.time().as_nanos(), jitter_ns);
            // A report that does not sit on the grid places no slots: it is
            // still counted as a report, and the gap it would have explained
            // stays unexplained rather than being explained by a guess.
            if off != 0 {
                continue;
            }
            for slot in cycle - i64::from(event.message.count())..cycle {
                missed.insert(slot);
            }
        }
        Self { events, missed }
    }

    /// The slots the reports account for, all told.
    #[must_use]
    pub fn slots(&self) -> u64 {
        self.events
            .iter()
            .map(|event| u64::from(event.message.count()))
            .sum()
    }

    /// Whether every cycle in `cycles` is one a report accounts for.
    ///
    /// An empty range is accounted for by nothing and answers true, which is
    /// what a caller asking about two consecutive samples wants.
    #[must_use]
    pub fn account_for(&self, cycles: std::ops::Range<i64>) -> bool {
        cycles.into_iter().all(|slot| self.missed.contains(&slot))
    }
}

#[cfg(test)]
mod tests {
    //! The grid arithmetic and the skip accounting, which is where the two
    //! analyzers agree about what a hole in the stream means.

    use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire};
    use clockwork_rs::SyncTime;
    use log_read::Logged;

    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, TickFaultWire};
    use reachy_motion::joints::{JointRef, ROW_COUNT, row, rows_of, write_rows};
    use run_report::Report;

    use reachy_motion::plant::{MAX_GAP_PERIODS, PlantModel, Predicted, RESPONSE_DEAD_SAMPLES};

    use super::{Grid, Skips, lags, no_faults, read_profile, residual_stream};

    /// A period nothing round, so an arithmetic that assumed one shows.
    const PERIOD_NS: i64 = 20_000_000;

    /// An origin nothing round, for the same reason.
    const ORIGIN_NS: i64 = 1_772_000_000_123_456_789;

    /// The grid a case reads instants against.
    fn grid() -> Grid {
        Grid {
            origin_ns: ORIGIN_NS,
            period_ns: PERIOD_NS,
        }
    }

    /// A skip report at cycle `n` accounting for `count` slots before it.
    fn skipped(n: i64, count: u32) -> Logged<DriverEventWire> {
        let mut message = DriverEventWire::new();
        message.set_kind(EventKindWire::CYCLE_SKIPPED);
        message.set_time(SyncTime::from_nanos(ORIGIN_NS + n * PERIOD_NS));
        message.set_count(count);
        Logged {
            at_ns: ORIGIN_NS + n * PERIOD_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// One sample: read at `present`, holding `commanded`.
    fn sample(
        n: i64,
        present: &[f64; ROW_COUNT],
        commanded: &[f64; ROW_COUNT],
    ) -> Logged<PoseSampleWire> {
        let mut message = PoseSampleWire::new();
        {
            let read = message.clear_valid();
            read.nominal_time = SyncTime::from_nanos(ORIGIN_NS + n * PERIOD_NS);
            read.sample_time = read.nominal_time;
            read.present_valid = true.into();
            read.commanded_valid = true.into();
            write_rows(&mut read.present, present);
            write_rows(&mut read.commanded, commanded);
        }
        Logged {
            at_ns: ORIGIN_NS + n * PERIOD_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// The head and the antennas are two figures, and which joint belongs to
    /// which group is the reason this crate exists: two analyzers print these
    /// numbers side by side, and a joint counted into the wrong group makes
    /// both reports consistently wrong.
    #[test]
    fn the_two_lag_figures_are_the_head_and_the_antennas_apart() {
        let mut present = [0.0; ROW_COUNT];
        present[row(JointRef::Leg2).expect("a bus row")] = 0.25;
        present[row(JointRef::AntennaLeft).expect("a bus row")] = 1.5;
        let samples = vec![
            sample(0, &present, &[0.0; ROW_COUNT]),
            sample(1, &present, &[0.0; ROW_COUNT]),
        ];
        let mut report = Report::default();
        lags(&samples, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst head lag 0.2500 rad")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst antenna lag 1.5000 rad")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("2 of 2 samples carried both")),
            "{:?}",
            report.measured
        );
    }

    /// A sample carrying no setpoint is compared against nothing, and the count
    /// says so: a run in which the driver held nothing would otherwise print
    /// the same zero as a run that tracked perfectly.
    #[test]
    fn a_sample_with_no_setpoint_is_not_compared() {
        let mut held = sample(1, &[0.5; ROW_COUNT], &[0.0; ROW_COUNT]);
        held.message.set_commanded_valid(false);
        let samples = vec![sample(0, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT]), held];
        let mut report = Report::default();
        lags(&samples, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 of 2 samples carried both")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst head lag 0.0000 rad")),
            "the uncompared sample's half-radian is not a lag: {:?}",
            report.measured
        );
    }

    /// A fault the tick raised is a finding carrying the kind, the instant and
    /// the window it was judged over.
    #[test]
    fn a_fault_is_reported_with_its_kind_its_instant_and_its_count() {
        let mut fault = TickFaultWire::new();
        fault.set_kind(FaultKindWire::HEAD_SERVO_FAULT);
        fault.set_time(SyncTime::from_nanos(ORIGIN_NS + 3 * PERIOD_NS));
        fault.set_count(4);
        let faults = vec![Logged {
            at_ns: ORIGIN_NS + 3 * PERIOD_NS,
            sequence_number: 0,
            message: fault,
        }];
        let mut report = Report::default();
        no_faults(&faults, &mut report);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        let said = &report.findings[0];
        assert!(said.contains("HEAD_SERVO_FAULT"), "{said}");
        assert!(
            said.contains(&(ORIGIN_NS + 3 * PERIOD_NS).to_string()),
            "{said}"
        );
        assert!(said.contains("count 4"), "{said}");
    }

    #[test]
    fn an_instant_on_the_grid_is_its_own_cycle_and_nothing_off_it() {
        assert_eq!(grid().at(ORIGIN_NS + 7 * PERIOD_NS), (7, 0));
        assert_eq!(grid().at(ORIGIN_NS + 7 * PERIOD_NS + 3), (7, 3));
    }

    #[test]
    fn an_instant_inside_the_jitter_band_is_on_the_cycle_it_is_nearest() {
        // Late for its own cycle, and early for the next one: both ends of the
        // band answer with a cycle and no offset.
        assert_eq!(grid().within(ORIGIN_NS + 7 * PERIOD_NS + 3, 5), (7, 0));
        assert_eq!(grid().within(ORIGIN_NS + 8 * PERIOD_NS - 3, 5), (8, 0));
        assert_eq!(
            grid().within(ORIGIN_NS + 7 * PERIOD_NS + 3, 0),
            (7, 3),
            "with no band allowed, an instant off the grid stays off it",
        );
    }

    #[test]
    fn a_report_accounts_for_the_slots_immediately_before_it() {
        let events = vec![skipped(10, 3)];
        let skips = Skips::of(&events, grid(), 0);
        assert_eq!(skips.slots(), 3);
        assert!(skips.account_for(7..10));
        assert!(
            !skips.account_for(6..10),
            "a gap wider than the report is a hole the driver did not explain",
        );
        assert!(!skips.account_for(10..11));
    }

    #[test]
    fn a_report_off_the_grid_places_no_slots() {
        let mut events = vec![skipped(10, 3)];
        events[0].message.set_time(SyncTime::from_nanos(
            ORIGIN_NS + 10 * PERIOD_NS + PERIOD_NS / 2,
        ));
        let skips = Skips::of(&events, grid(), 0);
        assert_eq!(skips.slots(), 3, "the report is still a report");
        assert!(
            !skips.account_for(7..10),
            "where the report sits is a guess, and a guess explains nothing",
        );
    }

    /// One sample holding no setpoint at all: the driver before its first goal,
    /// after a release, or behind a latched torque-off.
    fn holding_nothing(n: i64, present: &[f64; ROW_COUNT]) -> Logged<PoseSampleWire> {
        let mut logged = sample(n, present, &[0.0; ROW_COUNT]);
        logged.message.set_commanded_valid(false);
        logged
    }

    /// The row every residual case drives. One row moves and the other eight
    /// stand on a setpoint of zero, so a residual anywhere else is arithmetic
    /// leaking between rows.
    fn driven() -> usize {
        row(JointRef::BodyYaw).expect("a bus row")
    }

    /// A stream of samples in which the driven row reads exactly what the plant
    /// makes of the setpoints the driver held.
    ///
    /// The healthy machine, generated rather than recorded: the walk under test
    /// has to find no residual in it. `shift` offsets which setpoint the
    /// generated reading answers, which is how a case drives the dead time
    /// wrong on purpose -- at zero the reading answers the setpoint of
    /// [`RESPONSE_DEAD_SAMPLES`] samples ago, which is what the walk assumes.
    fn chase(plant: &PlantModel, commanded: &[f64], shift: usize) -> Vec<Logged<PoseSampleWire>> {
        let index = driven();
        let mut predicted = Predicted::default();
        let mut out = Vec::new();
        for (n, held) in commanded.iter().enumerate() {
            if n >= RESPONSE_DEAD_SAMPLES {
                plant.step(&mut predicted, commanded[n - RESPONSE_DEAD_SAMPLES + shift]);
            }
            let mut present = [0.0; ROW_COUNT];
            present[index] = predicted.position;
            let mut asked = [0.0; ROW_COUNT];
            asked[index] = *held;
            out.push(sample(
                i64::try_from(n).expect("a few samples"),
                &present,
                &asked,
            ));
        }
        out
    }

    /// A stream whose driven row does not move at all, under a setpoint a long
    /// way from it: the shape every gap case needs, because a walk that steps
    /// and a walk that re-seeds answer differently about it.
    fn frozen(count: i64, target: f64) -> Vec<Logged<PoseSampleWire>> {
        let index = driven();
        (0..count)
            .map(|n| {
                let mut asked = [0.0; ROW_COUNT];
                asked[index] = if n < 2 { 0.0 } else { target };
                sample(n, &[0.0; ROW_COUNT], &asked)
            })
            .collect()
    }

    /// The worst residual the driven row shows, and the worst any other row
    /// does.
    fn worsts(stream: &[(i64, [f64; ROW_COUNT])]) -> (f64, f64) {
        let index = driven();
        let mut driven_row = 0.0_f64;
        let mut elsewhere = 0.0_f64;
        for (_, residual) in stream {
            for (at, figure) in residual.iter().enumerate() {
                if at == index {
                    driven_row = driven_row.max(*figure);
                } else {
                    elsewhere = elsewhere.max(*figure);
                }
            }
        }
        (driven_row, elsewhere)
    }

    /// The residual a sample carries, found by its instant.
    fn at_cycle(stream: &[(i64, [f64; ROW_COUNT])], n: i64) -> f64 {
        let index = driven();
        stream
            .iter()
            .find(|(nominal, _)| *nominal == ORIGIN_NS + n * PERIOD_NS)
            .map(|(_, residual)| residual[index])
            .unwrap_or_else(|| panic!("the stream carries no sample for cycle {n}"))
    }

    /// The setpoints of a saturated move: further per period than the profile
    /// carries, which is what the recorded library is full of.
    fn saturated(count: usize) -> Vec<f64> {
        (0..count).map(|n| 0.1 * n as f64).collect()
    }

    /// The one assertion that separates this figure from the lag beside it: a
    /// machine following its own generator exactly reads a residual of zero
    /// while standing radians away from its goal.
    ///
    /// A walk that judged against the goal, or that returned all zeros, or that
    /// pushed before it read, could not both. And the number this pins is the
    /// evidence a run is judged by: an analyzer that always printed zero would
    /// read as a perfectly healthy tour.
    #[test]
    fn a_machine_on_its_own_trajectory_has_no_residual_however_far_behind_its_goal_it_is() {
        let plant = PlantModel::default();
        let commanded = saturated(60);
        let samples = chase(&plant, &commanded, 0);
        let stream = residual_stream(&samples, grid(), &plant);

        assert_eq!(stream.len(), samples.len(), "every sample was judged");
        let (driven_row, elsewhere) = worsts(&stream);
        assert!(driven_row < 1e-12, "the driven row's worst is {driven_row}");
        assert!(
            elsewhere < 1e-12,
            "and no other row's is anything: {elsewhere}"
        );

        // And the same samples are radians from their goal the whole way, which
        // is the figure `lags` prints and the reason it is not this one.
        let index = driven();
        let worst_lag = samples
            .iter()
            .map(|logged| {
                let read = logged.message.validate().expect("a fixture sample");
                (rows_of(&read.commanded)[index] - rows_of(&read.present)[index]).abs()
            })
            .fold(0.0_f64, f64::max);
        assert!(
            worst_lag > 1.0,
            "the content outran the profile by {worst_lag} rad"
        );
    }

    /// The dead time is pinned, not assumed: the same machine answering one
    /// period sooner than the model expects reads a residual of the order of
    /// the profile velocity.
    ///
    /// Which is what makes the case above an assertion about the dead time the
    /// model was fitted at rather than about any delay at all.
    #[test]
    fn a_reading_that_answers_a_period_early_reads_a_residual() {
        let plant = PlantModel::default();
        let commanded = saturated(60);
        let stream = residual_stream(&chase(&plant, &commanded, 1), grid(), &plant);
        let (driven_row, _) = worsts(&stream);
        assert!(
            driven_row > 0.5 * plant.v_max,
            "a period of a saturated move is {} rad, and the residual is {driven_row}",
            plant.v_max
        );
    }

    /// A grid slot no sample attended is stepped on the setpoint already held,
    /// because that is what the servos went on chasing.
    ///
    /// The stream is a healthy machine under a setpoint that does not change
    /// across the hole, so stepping through it is exactly right and not
    /// stepping through it leaves the prediction three periods behind a moving
    /// joint -- which is a residual, and would be one on the first live sample
    /// after every skipped cycle the driver reports.
    #[test]
    fn a_hole_in_the_grid_is_stepped_through_on_the_setpoint_the_driver_held() {
        let plant = PlantModel::default();
        // A step held from the third sample on, so the joint is still
        // travelling toward it when the hole falls.
        let commanded: Vec<f64> = (0..60).map(|n| if n < 2 { 0.0 } else { 2.0 }).collect();
        let samples = chase(&plant, &commanded, 0);
        let count = samples.len();
        let holed: Vec<_> = samples
            .into_iter()
            .filter(|logged| !(10..13).contains(&logged.sequence_number))
            .collect();
        assert_eq!(holed.len(), count - 3);

        let stream = residual_stream(&holed, grid(), &plant);
        let (driven_row, _) = worsts(&stream);
        assert!(
            driven_row < 1e-12,
            "the hole cost the prediction {driven_row} rad of the joint's own travel"
        );
    }

    /// Past the gap the model is walked through, the prediction is re-seeded
    /// from the reading rather than guessed at; up to it, it is stepped.
    ///
    /// The stream is a joint that never moves under a setpoint radians away, so
    /// the two answers are as far apart as they can be: a stepped walk measures
    /// the whole of the prediction's travel, and a re-seeded one measures
    /// nothing.
    #[test]
    fn a_gap_past_the_bound_re_seeds_and_one_inside_it_does_not() {
        let plant = PlantModel::default();
        // The gap is how many periods the sample after the hole covers: its
        // own, plus the slots nobody attended. So a gap of `n` is `n - 1`
        // samples missing.
        for gap in [MAX_GAP_PERIODS as i64, MAX_GAP_PERIODS as i64 + 1] {
            let hole = 20..20 + gap - 1;
            let holed: Vec<_> = frozen(60, 2.0)
                .into_iter()
                .filter(|logged| !hole.contains(&i64::from(logged.sequence_number)))
                .collect();
            let stream = residual_stream(&holed, grid(), &plant);
            let after = at_cycle(&stream, 19 + gap);
            if gap > MAX_GAP_PERIODS as i64 {
                assert_eq!(after, 0.0, "a gap of {gap} periods re-seeds");
            } else {
                assert!(
                    after > 0.1,
                    "a gap of {gap} periods is walked through, and the joint did not move: \
                     {after}"
                );
            }
        }
    }

    /// A sample the driver held no setpoint on leaves the model nothing to
    /// chase: the prediction re-seeds, and the samples until the ring is a dead
    /// time deep again measure nothing.
    #[test]
    fn a_driver_holding_nothing_re_seeds_the_walk() {
        let plant = PlantModel::default();
        let mut samples = frozen(60, 2.0);
        let index = driven();
        assert!(
            residual_stream(&samples, grid(), &plant)
                .iter()
                .any(|(_, residual)| residual[index] > 0.1),
            "the stream has to measure something for the re-seed to be visible"
        );

        samples[30] = holding_nothing(30, &[0.0; ROW_COUNT]);
        let stream = residual_stream(&samples, grid(), &plant);
        let dead = i64::try_from(RESPONSE_DEAD_SAMPLES).expect("a few samples");
        for n in 30..=30 + dead {
            assert_eq!(
                at_cycle(&stream, n),
                0.0,
                "cycle {n} is the re-seed and the ring filling behind it"
            );
        }
        assert!(
            at_cycle(&stream, 31 + dead) > 0.0,
            "and then the comparison is running again"
        );
    }

    /// The walk is over the grid and not over the log's order: a prediction is
    /// a walk along the periods, and a sample read out of turn would step it
    /// backwards.
    #[test]
    fn samples_out_of_order_walk_the_same_stream() {
        let plant = PlantModel::default();
        let samples = frozen(40, 2.0);
        let ordered = residual_stream(&samples, grid(), &plant);
        let mut shuffled = samples;
        shuffled.reverse();
        let reversed = residual_stream(&shuffled, grid(), &plant);
        assert_eq!(ordered.len(), reversed.len());
        for ((at, ours), (also, theirs)) in ordered.iter().zip(&reversed) {
            assert_eq!(at, also);
            assert_eq!(ours, theirs, "cycle {at}");
        }
    }

    /// The profile is read from the fields of the file and never from the prose
    /// around them, and a file that states no pair is a failure rather than a
    /// default.
    ///
    /// The comment case is the one that would be silent: `servo_profile`'s own
    /// comment block discusses the bench's pair, so a scan that took the first
    /// match anywhere would judge a log under a pair nobody commissioned.
    #[test]
    fn a_profile_is_read_from_its_fields_and_not_from_the_comments_around_them() {
        let dir = std::env::temp_dir();
        let write = |name: &str, text: &str| {
            let path = dir.join(format!("brenn-reachy-profile-{name}.textproto"));
            std::fs::write(&path, text).expect("a temporary file");
            path.to_string_lossy().to_string()
        };

        let commented = write(
            "commented",
            "# the bench ran profile_velocity: 600\n# and profile_acceleration: 400\n\
             profile_acceleration: 20\nprofile_velocity: 50\n",
        );
        assert_eq!(read_profile(&commented), Ok((20, 50)));

        let partial = write("partial", "profile_acceleration: 20\n");
        assert_eq!(
            read_profile(&partial),
            Err(format!("{partial} states no profile_velocity"))
        );

        let unreadable = write(
            "unreadable",
            "profile_acceleration: 20\nprofile_velocity: fast\n",
        );
        assert!(
            read_profile(&unreadable)
                .is_err_and(|says| says.contains("profile_velocity is no register value")),
            "{:?}",
            read_profile(&unreadable)
        );

        assert!(
            read_profile(&dir.join("nothing-here.textproto").to_string_lossy())
                .is_err_and(|says| says.contains("nothing-here")),
            "a file that is not there is the reason it is not there"
        );
    }
}
