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

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire, HealthReportWire};
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use dxl_proto::HardwareError;
use log_read::Logged;
use reachy_motion::arm::{Gains, GroupGains};
use reachy_motion::joints::{JointGroup, JointRef, Name, ROWS, group_of, row, rows_of};
use reachy_motion::plant::{
    GroupPlants, GroupProfiles, MAX_GAP_PERIODS, PROFILE_ACCELERATION_UNIT_RAD_PER_S2,
    PROFILE_VELOCITY_UNIT_RAD_PER_S, Predicted, ProfilePair, RESPONSE_DEAD_SAMPLES,
};
use reachy_motion::stillness::COUNT_RAD;
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

/// The directory, inside a run's records, holding the configuration files the
/// run was performed under.
pub const CONFIG_DIR: &str = "config";

/// The configuration files a run carries home, at their payload-relative
/// paths.
///
/// Three, and the same three the deploy overlay accepts: they are the files a
/// run can be varied by and the files an analyzer reads. The rest of the
/// payload's configuration is pinned by the scenario suite's parameter check
/// and read by nothing here, so a copy of it beside the records would be
/// weight without a reader.
///
/// Payload-relative *and* runfiles-relative, which is one path serving two
/// purposes: `<log>/config/<path>` is what the run ran on, and `<path>` under
/// an analyzer's own working directory is what the analyzing tree ships. The
/// difference between the two is what says an experiment overlay was in force.
pub const CONFIG_FILES: [&str; 3] = [
    "cogs/servo_profile.textproto",
    "cogs/servo_gains.textproto",
    "cogs/mover_params.textproto",
];

/// The configuration a run was performed under, read from the run's own
/// records.
///
/// Read from the log rather than from the analyzing tree, because those are
/// different machines' answers: the residual is the distance from a trajectory
/// the joint's class's own two registers define, so judging a log under any
/// other pair measures a machine nobody ran, and a tuning campaign varies these
/// files per run. The log is the ground truth of what ran.
pub struct RunConfig {
    /// The profile registers each class was commissioned with: acceleration
    /// first, then velocity, in register units.
    pub profiles: GroupProfiles,
    /// The position gains each class was commissioned with.
    pub gains: GroupGains,
    /// Whether the tracking detector judged this run.
    pub tracking_armed: bool,
    /// Each file's text as the run carried it, in [`CONFIG_FILES`] order, kept
    /// so the notes can say which of them the analyzing tree states
    /// differently.
    ///
    /// `None` is a configuration nobody read off a run's records -- the crafted
    /// runs of an analyzer's own tests -- and there is nothing to compare the
    /// tree against there.
    texts: Option<[String; CONFIG_FILES.len()]>,
}

/// The value of one flat `key: value` field, out of a file this repo writes.
///
/// A literal line scan and not a protobuf text parser: a dozen scalars in files
/// this repo writes are not a reason to implement one. Flat names and not
/// nested per-class messages for exactly that reason — a first-match scan over
/// a nested message would hand back the legs' velocity for the antennas'.
/// Comments are skipped rather than scanned because these files' comment blocks
/// discuss other values -- the bench's profile pair among them -- and a scan
/// that took the first match anywhere would read one of those as a value the
/// machine was commissioned with.
///
/// A comment after a value is cut off it. The loader that read this file on the
/// unit is protobuf's own text format, which accepts one, and a scan that
/// refused what the machine ran would lose a hardware round trip to a `#` --
/// after the run, with the fetched copy's digest already stamped.
fn field<'a>(text: &'a str, path: &str, name: &str) -> Result<&'a str, String> {
    Ok(text
        .lines()
        .filter(|line| !line.trim_start().starts_with('#'))
        .filter_map(|line| line.split_once(':'))
        .find(|(key, _)| key.trim() == name)
        .ok_or_else(|| format!("{path} states no {name}"))?
        .1
        .split('#')
        .next()
        .unwrap_or_default()
        .trim())
}

/// One register value's digits, decimal or protobuf's hexadecimal.
///
/// The loader accepts `0x14` for an integer field, so this does: the two
/// readings of one file disagreeing about what it says is the failure this
/// whole scan exists to avoid.
fn integer(value: &str, path: &str, name: &str, noun: &str) -> Result<u64, String> {
    let refuse = |error: std::num::ParseIntError| format!("{path}'s {name} is no {noun}: {error}");
    match value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        Some(digits) => u64::from_str_radix(digits, 16).map_err(refuse),
        None => value.parse().map_err(refuse),
    }
}

/// The three pairs stated by the text of a `servo_profile.textproto`.
fn profiles_of(text: &str, path: &str) -> Result<GroupProfiles, String> {
    let figure = |name: &str| -> Result<u32, String> {
        let value = integer(field(text, path, name)?, path, name, "register value")?;
        u32::try_from(value)
            .map_err(|_| format!("{path}'s {name} is {value}, which no register holds"))
    };
    // Keyed off the classes themselves, so the file's field names and the
    // classes have one statement between them: a spelling here and a spelling
    // in `JointGroup` that drifted apart would be a file this scan refuses at
    // analysis time rather than a build that fails.
    GroupProfiles::try_of_each(|group| {
        let class = group.config_prefix();
        Ok(ProfilePair {
            acceleration: figure(&format!("{class}_profile_acceleration"))?,
            velocity: figure(&format!("{class}_profile_velocity"))?,
        })
    })
}

/// The three triples stated by the text of a `servo_gains.textproto`.
fn gains_of(text: &str, path: &str) -> Result<GroupGains, String> {
    let figure = |name: &str| -> Result<u16, String> {
        let value = integer(field(text, path, name)?, path, name, "gain value")?;
        u16::try_from(value).map_err(|_| format!("{path}'s {name} is {value}, which no gain holds"))
    };
    GroupGains::try_of_each(|group| {
        let class = group.config_prefix();
        Ok(Gains {
            p: figure(&format!("{class}_p"))?,
            i: figure(&format!("{class}_i"))?,
            d: figure(&format!("{class}_d"))?,
        })
    })
}

/// Whether the text of a `mover_params.textproto` armed the tracking detector.
///
/// Every spelling protobuf's text format takes for a boolean, because the unit
/// took them all: a run recorded as disarmed on the strength of a `True` this
/// scan could not read would be the analyzer disagreeing with the machine about
/// the one field that says whether anything judged the joints.
fn tracking_armed_of(text: &str, path: &str) -> Result<bool, String> {
    match field(text, path, "tracking_armed")? {
        "true" | "True" | "t" | "1" => Ok(true),
        "false" | "False" | "f" | "0" => Ok(false),
        other => Err(format!(
            "{path}'s tracking_armed is {other}, which is neither true nor false"
        )),
    }
}

impl RunConfig {
    /// What the run under `log_dir` was performed under, out of the `config/`
    /// directory beside its records.
    ///
    /// # Errors
    ///
    /// That the directory is not there — a log recorded by a payload that did
    /// not carry the copy, which is a log this analyzer refuses rather than
    /// judges against the analyzing tree's own files — or the reason one of the
    /// three files could not be read or does not state a name.
    pub fn read(log_dir: &Path) -> Result<Self, String> {
        let root = log_dir.join(CONFIG_DIR);
        if !root.is_dir() {
            return Err(format!(
                "there is no configuration beside these records: {} holds no {CONFIG_DIR}/ saying \
                 which profile, gains and mover parameters the run was performed under",
                log_dir.display()
            ));
        }
        let mut texts = [const { String::new() }; CONFIG_FILES.len()];
        for (text, name) in texts.iter_mut().zip(CONFIG_FILES) {
            let path = root.join(name);
            *text = std::fs::read_to_string(&path)
                .map_err(|error| format!("{}: {error}", path.display()))?;
        }
        let named = |index: usize| root.join(CONFIG_FILES[index]).display().to_string();
        Ok(Self {
            profiles: profiles_of(&texts[0], &named(0))?,
            gains: gains_of(&texts[1], &named(1))?,
            tracking_armed: tracking_armed_of(&texts[2], &named(2))?,
            texts: Some(texts),
        })
    }

    /// A configuration stated rather than read: what an analyzer's own crafted
    /// runs are judged under.
    ///
    /// No file texts, so nothing is said about how the tree differs from it.
    /// Off a machine's records the whole point is that those texts exist, which
    /// is why [`Self::read`] is the only way a real run gets one.
    #[must_use]
    pub fn stated(profiles: GroupProfiles, gains: GroupGains, tracking_armed: bool) -> Self {
        Self {
            profiles,
            gains,
            tracking_armed,
            texts: None,
        }
    }

    /// What the run was configured with, and whether that configuration is the
    /// one this tree ships.
    ///
    /// The disarmed detector is a `fail` and has no flag to excuse it: the log
    /// is the ground truth of what ran and the verdict is a function of the log
    /// alone, so a run whose detector was not judging never reads green. The
    /// one run that is meant to be disarmed — a capability measurement at a
    /// profile the motors cannot follow — is read for its notes, and its record
    /// says so.
    ///
    /// The tree comparison is a note rather than a verdict, because an overlay
    /// is a legitimate thing to run: what it must never be is invisible in the
    /// report of the run it shaped.
    pub fn configuration(&self, report: &mut Report) {
        report.note(format!(
            "configuration {}: legs {:?}, body yaw {:?}, antennas {:?}, acceleration first, in \
             register units",
            CONFIG_FILES[0], self.profiles.legs, self.profiles.yaw, self.profiles.antennas,
        ));
        report.note(format!(
            "configuration {}: legs {}, body yaw {}, antennas {}",
            CONFIG_FILES[1], self.gains.legs, self.gains.yaw, self.gains.antennas,
        ));
        report.note(format!(
            "configuration {}: tracking_armed: {}",
            CONFIG_FILES[2], self.tracking_armed,
        ));
        for (text, name) in self.texts.iter().flatten().zip(CONFIG_FILES) {
            match std::fs::read_to_string(name) {
                Ok(ours) if &ours == text => {}
                Ok(_) => report.note(format!(
                    "this run's {name} is not the tree's, so it ran under an experiment overlay \
                     and its figures describe that configuration"
                )),
                Err(error) => report.note(format!(
                    "this tree's own {name} could not be read, so whether the run's copy differs \
                     from it is unknown: {error}"
                )),
            }
        }
        if !self.tracking_armed {
            report.fail(
                "this run's tracking detector was disarmed, so nothing was judging any joint \
                 against its own servo's trajectory: the run measures capability and is not a \
                 run that can pass"
                    .to_string(),
            );
        }
    }
}

/// How far one reading stood from its own model's prediction, and which side of
/// the model it stood on.
///
/// A type and not a signed number, because the sign is a different fact from
/// the magnitude rather than a direction of the same one. The magnitude is the
/// disagreement the tracking screen is sized on; the side says whether the
/// joint stood behind its own trajectory -- no nearer the goal it was
/// answering than its prediction stood -- or ahead of it, and the two mean
/// opposite things for a candidate profile pair. A caller reading the stored
/// number as a distance would screen a joint behind its model at the wrong
/// sign and see a clean report, so the number is not readable as one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Residual(f64);

impl Residual {
    /// A reading `distance` rad from its prediction, standing no nearer the
    /// goal the two were answering than the prediction did.
    #[must_use]
    pub fn behind(distance: f64) -> Self {
        Self(-distance.abs())
    }

    /// A reading `distance` rad from its prediction, standing nearer that goal.
    #[must_use]
    pub fn ahead(distance: f64) -> Self {
        Self(distance.abs())
    }

    /// What a sample the walk measured nothing at carries: no disagreement and
    /// so no side either.
    #[must_use]
    pub fn unmeasured() -> Self {
        Self(0.0)
    }

    /// How far the reading and the prediction disagree, radians, whichever
    /// side the reading stood.
    #[must_use]
    pub fn magnitude(self) -> f64 {
        self.0.abs()
    }

    /// Whether the joint stood behind its own trajectory.
    #[must_use]
    pub fn is_behind(self) -> bool {
        self.0 < 0.0
    }

    /// Whether it stood ahead of it.
    #[must_use]
    pub fn is_ahead(self) -> bool {
        self.0 > 0.0
    }
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
/// and the nine [`Residual`]s. A sample the walk measured nothing at -- one
/// that re-seeded, or one the ring was still filling behind -- carries
/// [`Residual::unmeasured`].
#[must_use]
pub fn residual_stream(
    samples: &[Logged<PoseSampleWire>],
    grid: Grid,
    plant: &GroupPlants,
) -> Vec<(i64, [Residual; ROWS.len()])> {
    // Nominal order is the model's own order, whatever order the log holds:
    // the prediction is a walk along the grid and a sample read out of turn
    // would step it backwards.
    let mut ordered: Vec<&Logged<PoseSampleWire>> = samples.iter().collect();
    ordered.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
    let mut predicted = [Predicted::default(); ROWS.len()];
    let mut seeded = [false; ROWS.len()];
    let mut ring: Vec<[f64; ROWS.len()]> = Vec::new();
    // The goal each row's model was last stepped toward, which is the goal the
    // servo was answering: a residual's sign is whether the reading or the
    // prediction stood nearer it, so the sign needs the goal and not just the
    // two positions.
    let mut answering = [f64::NAN; ROWS.len()];
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
                        plant.for_row(index).step(state, target[index]);
                        answering[index] = target[index];
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
        let mut residual = [Residual::unmeasured(); ROWS.len()];
        for (index, state) in predicted.iter_mut().enumerate() {
            if !seeded[index] {
                state.position = present[index];
                state.velocity = 0.0;
                seeded[index] = true;
                answering[index] = f64::NAN;
            }
            let disagreement = (present[index] - state.position).abs();
            // Behind where the joint stood no nearer the goal than its
            // prediction did, which includes the two standing equally near and
            // a prediction that has already arrived: nothing can be nearer a
            // goal the model is sitting on. A row with no goal on record is a
            // row the walk stepped nothing for, and its disagreement is zero.
            let goal = answering[index];
            let nearer =
                goal.is_finite() && (goal - present[index]).abs() < (goal - state.position).abs();
            residual[index] = if disagreement == 0.0 {
                Residual::unmeasured()
            } else if nearer {
                Residual::ahead(disagreement)
            } else {
                Residual::behind(disagreement)
            };
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
    stream: &[(i64, [Residual; ROWS.len()])],
    samples: &[Logged<PoseSampleWire>],
    plant: &GroupPlants,
    report: &mut Report,
) {
    let threshold = TrackingFaultConfig::default().threshold_rad;
    report.note(format!(
        "{} of {} samples were judged against the modelled trajectory of the servo's own class",
        stream.len(),
        samples.len(),
    ));
    // One walk, partitioned into the three classes as it goes, which is the
    // shape the capability pass below uses: three walks of the whole stream
    // would read the same nine rows three times over the longest run this repo
    // takes.
    let mut seen: [Vec<f64>; JointGroup::ALL.len()] = Default::default();
    let mut behind: [f64; JointGroup::ALL.len()] = Default::default();
    let mut ahead: [f64; JointGroup::ALL.len()] = Default::default();
    for (_, residual) in stream {
        for joint in ROWS {
            let (Some(index), Some(group)) = (row(joint), group_of(joint)) else {
                continue;
            };
            let reading = residual[index];
            seen[slot(group)].push(reading.magnitude());
            if reading.is_behind() {
                behind[slot(group)] = behind[slot(group)].max(reading.magnitude());
            } else if reading.is_ahead() {
                ahead[slot(group)] = ahead[slot(group)].max(reading.magnitude());
            }
        }
    }
    for group in JointGroup::ALL {
        let model = plant.of(group);
        let ranked = &mut seen[slot(group)];
        ranked.sort_by(f64::total_cmp);
        let worst = ranked.last().copied().unwrap_or(0.0);
        let p999 = percentile(ranked, 0.999);
        let (low, high) = recorded_p999_range(group);
        let configuration = recorded_p999_configuration(group);
        report.note(format!(
            "{}: worst residual {worst:.4} rad, p99.9 {p999:.4} rad, recorded p99.9 \
             {low:.4}–{high:.4} rad over three tours {configuration}, against a tracking screen \
             at {threshold:.4} rad — commissioned at {:.6} rad/period and {:.6} rad/period²",
            group.name(),
            model.v_max,
            model.a_max,
        ));
        // The same worst, split by which way the joint stood off its model,
        // because the two mean opposite things for a candidate pair: a joint
        // behind its own generator is answered by a slower pair, and a joint
        // ahead of one is a machine that outran a model derived as its floor,
        // which stepping the pair down would only widen.
        report.note(format!(
            "{}: worst {:.4} rad behind its own trajectory, worst {:.4} rad ahead of it",
            group.name(),
            behind[slot(group)],
            ahead[slot(group)],
        ));
    }
    report.note(format!(
        "the recorded library ran at {RECORDED_WORST_HEAD_RESIDUAL_RAD:.4} rad on the head and \
         {RECORDED_WORST_ANTENNA_RESIDUAL_RAD:.4} rad on the antennas, which is what the screen \
         is sized over"
    ));
}

/// The value at `fraction` of the way up `values`, by nearest rank, or zero if
/// there are none.
///
/// `values` is already sorted ascending, and the caller sorts it: every caller
/// asks for two or three ranks off one series, and a function that sorted would
/// re-sort an ordered slice once per rank.
fn percentile(values: &[f64], fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let rank = (fraction * values.len() as f64).ceil() as usize;
    values[rank.saturating_sub(1).min(values.len() - 1)]
}

/// What the health rotation saw, per servo, over the whole run.
///
/// The rotation visits one servo at a time and every visit is a reading, so a
/// run holds a series per servo rather than a picture of the machine. What is
/// printed is that series reduced: the first and last temperature, the peak and
/// how far into the run it was reached -- seconds from the run's first health
/// reading, because a thermal peak is read against what the machine was playing
/// at the time -- the voltage range, and the worst error byte any reading
/// carried. The last reading alone would say a servo that ran hot in the middle
/// and cooled by the end was never hot, which is the question a capability run
/// is taken to answer.
///
/// One finding for the whole set rather than one per servo. A bus-wide
/// condition is one fact about the machine, and nine copies of it bury the rest
/// of the report. Which servos, and what each of them latched, is in the line.
///
/// The input-voltage bit on its own is the exception, and it is not a finding
/// on this machine: the servo bus rail is specified above the highest Max
/// Voltage Limit the register accepts, so a healthy unit sets that bit by
/// arithmetic. `dxl_proto::HardwareError` is the one predicate that says so and
/// this pass judges through it. The bit is never filtered away -- every servo's
/// byte is printed, and the set that latched it is named in a note of its own.
/// Any bit beyond input-voltage, on any servo, is a finding, and the byte is
/// named whole, so a voltage bit riding alongside an overload launders nothing.
///
/// Temperature is a verdict: a servo whose peak reached [`TEMPERATURE_STOP_C`]
/// fails the run, one finding naming every such servo with its peak and when it
/// was reached. The figure came off three healthy library tours at the shipped
/// pair, whose hottest servo reached 37 C; the constant's own comment carries
/// the derivation and the margin. The servo's own overheating bit is in the
/// error byte and is judged with the rest, which is the ceiling this one sits
/// under.
pub fn health_summary(readings: &[Logged<HealthReportWire>], report: &mut Report) {
    /// One servo's series, reduced as it is read.
    struct Seen {
        first_at: i64,
        first_temp: i8,
        last_at: i64,
        last_temp: i8,
        peak_temp: i8,
        peak_at: i64,
        low_volts: f64,
        high_volts: f64,
        bits: u8,
    }
    let mut seen: BTreeMap<u8, Seen> = BTreeMap::new();
    for reading in readings {
        let message = &reading.message;
        let at = message.sample_time().as_nanos();
        let entry = seen.entry(message.id()).or_insert_with(|| Seen {
            first_at: at,
            first_temp: message.temp_c(),
            last_at: at,
            last_temp: message.temp_c(),
            peak_temp: message.temp_c(),
            peak_at: at,
            low_volts: message.volts(),
            high_volts: message.volts(),
            bits: 0,
        });
        if at <= entry.first_at {
            entry.first_at = at;
            entry.first_temp = message.temp_c();
        }
        if at >= entry.last_at {
            entry.last_at = at;
            entry.last_temp = message.temp_c();
        }
        if message.temp_c() > entry.peak_temp {
            entry.peak_temp = message.temp_c();
            entry.peak_at = at;
        }
        entry.low_volts = entry.low_volts.min(message.volts());
        entry.high_volts = entry.high_volts.max(message.volts());
        // The worst byte over the run, not the last one: a servo that latched
        // an overload and was read again after a power cycle would otherwise
        // report clean. Bit-wise, so two readings latching different bits are
        // both carried.
        entry.bits |= message.bits();
    }
    if seen.is_empty() {
        report.note("the health rotation reported nothing".to_string());
        return;
    }
    // The instant every time in this section is stated against: the first
    // health reading of the run. An epoch nanosecond is not a figure an
    // operator can put beside what the machine was playing, and the run's own
    // origin is what the rest of these reports count from.
    let origin = seen.values().map(|servo| servo.first_at).min().unwrap_or(0);
    let seconds = |at: i64| (at - origin) as f64 * 1e-9;
    let mut complaining: Vec<String> = Vec::new();
    let mut voltage_only: Vec<String> = Vec::new();
    let mut hot: Vec<String> = Vec::new();
    for (id, servo) in &seen {
        report.note(format!(
            "servo {id}: {:.2} V to {:.2} V, {} C at the first reading and {} C at the last, peak \
             {} C at {:.1} s into the run, worst error byte 0x{:02x}",
            servo.low_volts,
            servo.high_volts,
            servo.first_temp,
            servo.last_temp,
            servo.peak_temp,
            seconds(servo.peak_at),
            servo.bits
        ));
        let latched = HardwareError(servo.bits);
        if latched.bits_other_than_voltage() != 0 {
            complaining.push(format!("servo {id} (0x{:02x})", servo.bits));
        } else if servo.bits != 0 {
            voltage_only.push(format!("servo {id}"));
        }
        if servo.peak_temp >= TEMPERATURE_STOP_C {
            hot.push(format!(
                "servo {id} ({} C at {:.1} s)",
                servo.peak_temp,
                seconds(servo.peak_at)
            ));
        }
    }
    if !voltage_only.is_empty() {
        report.note(format!(
            "the input-voltage bit is latched on {} of the servos the health rotation read: {} -- \
             expected on this machine, where the rail is specified above the register's Max \
             Voltage Limit, so a healthy unit sets that bit by arithmetic",
            voltage_only.len(),
            voltage_only.join(", ")
        ));
    }
    if !complaining.is_empty() {
        report.fail(format!(
            "the run latched an error byte on {} of the servos the health rotation read: {}",
            complaining.len(),
            complaining.join(", ")
        ));
    }
    if !hot.is_empty() {
        report.fail(format!(
            "{} of the servos the health rotation read reached {TEMPERATURE_STOP_C} C, which no \
             healthy tour of the whole library has come near: {}",
            hot.len(),
            hot.join(", ")
        ));
    }
}

/// How far a setpoint has to stand from where the joint is for the joint to be
/// chasing it, radians.
///
/// Past it the joint is moving as fast as it is going to, so the travel it makes
/// in that period is what its motor achieves rather than what the content asked
/// for. Under it the joint is arriving, and its travel says only how close it
/// already was.
///
/// Also the width of the error bins the travel table is read over
/// ([`ClassCapability::bins`]) and the smallest goal change the step listing
/// counts as a step: one gap is one bin, so the first bin starts where a chase
/// starts and a bin's own name is the error it was measured at.
pub const CHASE_GAP_RAD: f64 = 0.1;

/// How many periods after a goal step the listing prints travel over.
///
/// The listing's width and nothing else. The step's ramp is read off the first
/// two of them, and the four after those are printed so a person can see the
/// cruise the ramp ran into and disagree with the reading. Six periods reach a
/// tenth of a second past the write, which is longer than any ramp on this
/// machine has taken.
pub const STEP_LISTING_PERIODS: usize = 6;

/// How many chasing samples an error bin needs before its percentiles are read.
///
/// Under it the bin is printed and marked unread rather than dropped: an
/// absence is a reading too -- it says the content never held this class that
/// far behind -- and a median over a handful of samples is the shape of one
/// motion. Twenty is where a p10 and a p90 stop being the same two samples.
pub const CAPABILITY_BIN_MIN_SAMPLES: usize = 20;

/// How close two adjacent bins' median travel has to be for the speed between
/// them to count as having stopped rising.
///
/// The one figure the regime reading turns on. A proportional loop's speed
/// grows with its error until the motor is the binding constraint, and then it
/// does not: two bins within this ratio are two errors the class answered at
/// the same speed. 1.15 is over the run-to-run spread the same tour played
/// twice shows (worst 1.07 across the two capability tours) and under the step
/// a gain-bound class makes from one bin to the next.
pub const CAPABILITY_PLATEAU_RATIO: f64 = 1.15;

/// How many goal steps a class needs before the median of their ramps is read
/// as the class's acceleration.
///
/// A median over fewer is the shape of one or two motions rather than the
/// motor's: the ramp a step shows depends on how far the step reached, so the
/// statistic needs steps of several sizes under it. A class below this prints
/// its listing and no acceleration figure at all.
pub const CAPABILITY_STEP_MIN_SAMPLES: usize = 10;

/// The Velocity Limit register the read-only self-test recorded on the seven
/// head servos, in Profile Velocity units.
///
/// Read off this machine's self-test, not the datasheet. Printed beside the
/// measured travel
/// because that is what tells a motor that reached its own limit from one that
/// fell short of it -- the profile the generator runs cannot be written above
/// it, so a class travelling at it is a class with no headroom left.
pub const RECORDED_VELOCITY_LIMIT_HEAD: u32 = 445;

/// The same register on the two antennas, which are a different XL330 variant.
pub const RECORDED_VELOCITY_LIMIT_ANTENNAS: u32 = 1620;

/// What the legs achieved on the capability tour, as a profile pair: Profile
/// Acceleration then Profile Velocity, the order the configuration writes.
///
/// Read by this file's own instrument off the kept capability tour
/// `tour-log-20260908T020318Z`, played at `(32767, 445)` so that the motor and
/// not the generator was the binding constraint. The class read motor-bound:
/// the velocity is the smallest median in the plateau bins and the
/// acceleration is the median of the tour's goal-step ramps. The repeat tour
/// `tour-log-20260908T031159Z` read `(307, 326)` by the same instrument, which
/// is a ratio of 1.07 on the acceleration and equality on the velocity,
/// against a repeatability gate of [`CAPABILITY_PLATEAU_RATIO`].
///
/// Commissioned: this is the pair `SHIPPED_PROFILES.legs` now carries, and a
/// confirmation tour of the whole library at it held the six cranks 0.3319 rad
/// from their own modelled trajectory at the worst, against a 0.4 rad bound;
/// provenance in `docs/servo-tuning.md`.
///
/// That tour re-read the class motor-bound with the plateau's slowest median at
/// 326 units, the recorded velocity to the digit. It read the goal-step ramp
/// median at 82 units, and that figure is not a reading of this class: the
/// steps in it are the library's own 0.10–0.23 rad leg moves under a generator
/// commissioned at 287, so the ramp measured is the content's demand and the
/// generator was never the binding constraint on it. The instrument cannot read
/// a motor's acceleration through a generator written at it, which is why the
/// acceleration here stays the wide-open tour's and the confirmation tour's
/// figure is a note in `docs/servo-tuning.md`. The velocity does not suffer
/// that, because the content does saturate the velocity.
pub const RECORDED_CAPABILITY_LEGS: ProfilePair = ProfilePair {
    acceleration: 287,
    velocity: 326,
};

/// The same reading for the two antennas, also motor-bound on that tour; the
/// repeat tour read `(522, 611)`, equality on the acceleration and 1.05 on the
/// velocity.
///
/// The campaign's own offline script read this acceleration as 532 over 13
/// goal steps on the kept tour and 15 on the repeat, where the instrument here
/// finds 18 and 16 and reads 522 on both. The two readings differ by 1.02,
/// inside the repeatability ratio, and the figure this file bakes is the
/// instrument's: it is the one a re-read reproduces.
///
/// On record and *not* commissioned, unlike the legs': at the shipped
/// `200 / 0 / 0` gains an antenna played at this pair reaches its generator's
/// speed — three confirmation tours read the class motor-bound with a plateau
/// above the commissioned velocity — but follows it 1.45 to 1.94 periods of
/// travel behind, 0.42 to 0.51 rad, and the tracking screen is sized at half
/// again the worst residual a healthy machine shows. So the pair the motor can
/// do is faster than the plant model can be right at, and what stands between
/// the two is a model that carries that following lag.
/// `TODO(session-servo-profile)` holds the question.
pub const RECORDED_CAPABILITY_ANTENNAS: ProfilePair = ProfilePair {
    acceleration: 522,
    velocity: 640,
};

/// The same reading for the body yaw, which read *gain-bound* on both tours:
/// its velocity is the median of the bin two gaps out rather than a plateau's,
/// and its acceleration is the median increase, because one goal step is fewer
/// than [`CAPABILITY_STEP_MIN_SAMPLES`] and no ramp median exists. The
/// acceleration is the shipped value, which is the reading: what holds this
/// class back is its loop, not its motor.
pub const RECORDED_CAPABILITY_BODY_YAW: ProfilePair = ProfilePair {
    acceleration: 20,
    velocity: 48,
};

/// The capability pair recorded for one class — the other figure, beside
/// [`ClassCapability::recorded_velocity_limit`], that a fresh reading is
/// compared against.
#[must_use]
pub fn recorded_capability(group: JointGroup) -> ProfilePair {
    match group {
        JointGroup::BodyYaw => RECORDED_CAPABILITY_BODY_YAW,
        JointGroup::Legs => RECORDED_CAPABILITY_LEGS,
        JointGroup::Antennas => RECORDED_CAPABILITY_ANTENNAS,
    }
}

/// The temperature at which a run stops reading as healthy, degrees Celsius.
///
/// Derived, not assumed. The hottest reading a healthy tour of the whole
/// library at the shipped `20 / 50` pair has produced is 37 C (servo 18), at
/// an ambient nobody recorded, and a whole tour raises a servo 1 to 3 C.
/// Provenance in `docs/servo-tuning.md`. 50 C is 13 C over that peak, more
/// than four tours' worth of heating in one run, so a machine reaching it is
/// doing something the library has never made it do; and it is 20 C under the
/// Temperature Limit register's 70 C, so the reading is a verdict here before
/// the servo's own protection acts through the overheating bit.
///
/// Two jobs. It fails a run in [`health_summary`], on any servo, whatever the
/// run was for. And it is the ceiling the capability tour is read against: a
/// class whose servo reaches it cannot play the library at its own speed, and
/// its candidate pair is bounded by the pace at which the temperature was still
/// flat. The per-servo *rise* is read on that tour too, against the healthy 1
/// to 3 C, but it is recorded and not judged — no aggressive tour has been
/// measured, so there is no figure to set a rise threshold against.
///
/// An analyzer figure, which is why it sits beside the recorded Velocity
/// Limits and not in the detector's configuration: nothing on the machine reads
/// it, and no motion is stopped by it.
pub const TEMPERATURE_STOP_C: i8 = 50;

/// One band of tracking error, and the travel the class made while it stood
/// that far behind.
///
/// The reading that separates a motor from a loop. A proportional controller's
/// speed is its gain times its error, so a class whose bins keep rising is a
/// class whose gain decided the speed; a class whose top bins read the same
/// speed has reached what its motor does.
pub struct ErrorBin {
    /// The band's own error range, radians: `low` up to but not including
    /// `high`, one [`CHASE_GAP_RAD`] wide.
    pub low: f64,
    /// The top of the band, radians.
    pub high: f64,
    /// How many chasing (sample, joint) pairs fell in it.
    pub samples: usize,
    /// The per-period travel over them, radians: a tenth were slower.
    pub travel_p10: f64,
    /// The median per-period travel in the band, radians. The figure the
    /// regime and the plateau are read off.
    pub travel_p50: f64,
    /// The travel a tenth of the band's samples beat, radians.
    pub travel_p90: f64,
}

impl ErrorBin {
    /// Whether the band holds enough samples for its percentiles to be read.
    #[must_use]
    pub fn readable(&self) -> bool {
        self.samples >= CAPABILITY_BIN_MIN_SAMPLES
    }
}

/// A readable band the regime reading was taken under, because it came out
/// slower than the band below it by more than [`CAPABILITY_PLATEAU_RATIO`].
///
/// Not a speed the class held: a motor at its ceiling holds a speed as the
/// error grows rather than losing it, so a band that fell is the periods after
/// a large goal step -- dead time and ramp, while the joint is a whole move
/// behind -- or a stall. Printed with its own figures so a person can disagree
/// with the walk.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct FellAway {
    /// The band's own error range, radians.
    pub low: f64,
    /// The top of the band, radians.
    pub high: f64,
    /// How many chasing (sample, joint) pairs fell in it.
    pub samples: usize,
    /// The band's median per-period travel, radians.
    pub travel_p50: f64,
    /// The median of the band under it, radians per period: the figure it fell
    /// away from.
    pub under: f64,
}

impl FellAway {
    /// How far under the band below it this band's median sits, where that is
    /// a number.
    ///
    /// [`None`] where the band read zero: the ratio is not a number, and the
    /// report says the band stalled instead of printing an infinity.
    #[must_use]
    pub fn ratio(&self) -> Option<f64> {
        (self.travel_p50 > 0.0).then(|| self.under / self.travel_p50)
    }
}

/// What the travel table says was holding a class back.
///
/// A note and never a verdict: the report prints the name with the two figures
/// it was read off, so a person can disagree with it. What it decides is which
/// rule the candidate pair comes off, and that decision is theirs. Read over
/// the bands below any fall: a band set aside as [`FellAway`] is no part of it.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Regime {
    /// The two highest bands left read the same speed: the class's speed
    /// stopped rising with its error, so the motor is the binding constraint.
    /// It carries the plateau, which is the only regime a plateau is a reading
    /// of: a candidate velocity comes off it, and the other two regimes have
    /// no speed the class held to take one from.
    MotorBound(Plateau),
    /// The highest band left is still faster than the one below it by more
    /// than [`CAPABILITY_PLATEAU_RATIO`]: the speed is the loop's gain times
    /// the error, and the motor was never reached.
    GainBound,
    /// Fewer than two bands left, so there is no pair of speeds to compare and
    /// nothing says whether the class was ever the binding constraint. Where
    /// bands fell away to reach it, the run is no measurement of the class at
    /// all: it held the class far behind, and the far bands are the ones that
    /// were set aside.
    ContentBound,
}

impl Regime {
    /// What the regime is called in a report line.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::MotorBound(_) => "motor-bound",
            Self::GainBound => "gain-bound",
            Self::ContentBound => "content-bound",
        }
    }
}

/// The run of top bins whose speeds are one speed, and the slowest of them.
///
/// The slowest and not the fastest: a candidate Profile Velocity is the speed
/// the loaded motor *holds* across the errors it cruises at, so a pair written
/// from the plateau's top would be a pair the class only reaches at its widest
/// error.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Plateau {
    /// The bottom of the lowest bin in the run, radians.
    pub low: f64,
    /// The top of the highest, radians.
    pub high: f64,
    /// How many bins the run holds.
    pub bins: usize,
    /// The smallest median travel among them, radians per period.
    pub travel: f64,
}

/// One goal change of more than a chase gap, written while the joint stood
/// still, and what the joint did after it.
///
/// A listing and not a statistic: it is the direct reading of the dead time --
/// how many periods the joint stood after the write -- and of the acceleration
/// the motor ramps at, and both are things a person checks by looking rather
/// than by reading a median.
pub struct GoalStep {
    /// Which joint the goal was written for.
    pub joint: JointRef,
    /// The instant of the sample the goal changed at, nanoseconds, so the step
    /// can be handed to `//cogs:trace_export`.
    pub at_ns: i64,
    /// How far the goal moved in that one period, radians.
    pub step: f64,
    /// The joint's travel over the periods after the write, radians per
    /// period, starting with the period after it.
    pub travel: Vec<f64>,
    /// The second post-write period's travel less the first's, radians per
    /// period squared, or nothing where the stretch ended before both.
    ///
    /// The step's ramp: at a dead time of two periods the joint starts late in
    /// the first period after the write and the second is its first whole
    /// period of acceleration, so this is the trapezoid's own first ramp
    /// period rather than a figure spread over a window.
    pub ramp: Option<f64>,
}

/// What one class of servo achieved over a run, in the units its own two
/// profile registers are written in.
///
/// Measured over the samples where the class was chasing a setpoint far enough
/// away to be moving as fast as it can (see [`CHASE_GAP_RAD`]): a joint arriving
/// at a goal travels slowly because it is nearly there, and averaging that in
/// would report a motor slower than the one in the machine.
pub struct ClassCapability {
    /// Which class these figures are about.
    pub group: JointGroup,
    /// How many (sample, joint) pairs the travel figures came off.
    pub chasing: usize,
    /// The per-period travel over those pairs, radians: a tenth of them were
    /// slower than this.
    pub travel_p10: f64,
    /// The median per-period travel, radians.
    pub travel_p50: f64,
    /// The per-period travel a tenth of them were faster than, radians.
    pub travel_p90: f64,
    /// The fastest single period the class ran, radians.
    pub travel_max: f64,
    /// The same travel cut by how far behind the joint stood, lowest band
    /// first.
    pub bins: Vec<ErrorBin>,
    /// What the bins say was holding the class back, and -- where that is the
    /// motor -- the plateau its candidate velocity comes off.
    pub regime: Regime,
    /// The top bands the regime was read under, fastest first, because each
    /// lost speed against the band below it.
    pub fell_away: Vec<FellAway>,
    /// How many chasing periods travelled at least an encoder count further
    /// than the period before them.
    pub increases: usize,
    /// The median of those gains, radians per period squared.
    pub increase_p50: f64,
    /// The gain a tenth of them beat, radians per period squared.
    pub increase_p90: f64,
    /// Every goal step the class was written, in run order.
    pub steps: Vec<GoalStep>,
    /// How many samples found the class chasing on a non-finite reading, which
    /// is corruption in the log rather than a speed, and measure nothing.
    pub unreadable: usize,
    /// The median step ramp, radians per period squared, where the class was
    /// written at least [`CAPABILITY_STEP_MIN_SAMPLES`] steps that carried one.
    pub step_ramp_p50: Option<f64>,
    /// The ramp a tenth of the steps beat, on the same condition.
    pub step_ramp_p90: Option<f64>,
    /// The grid these figures are per-period of, so they can be stated in
    /// register units.
    pub period_ns: i64,
}

impl ClassCapability {
    /// A per-period travel in Profile Velocity register units.
    #[must_use]
    pub fn velocity_units(&self, rad_per_period: f64) -> f64 {
        let period_s = self.period_ns as f64 * 1e-9;
        rad_per_period / (PROFILE_VELOCITY_UNIT_RAD_PER_S * period_s)
    }

    /// A per-period increase in travel in Profile Acceleration register units.
    #[must_use]
    pub fn acceleration_units(&self, rad_per_period2: f64) -> f64 {
        let period_s = self.period_ns as f64 * 1e-9;
        rad_per_period2 / (PROFILE_ACCELERATION_UNIT_RAD_PER_S2 * period_s * period_s)
    }

    /// The Velocity Limit this class's servos were recorded at.
    #[must_use]
    pub fn recorded_velocity_limit(&self) -> u32 {
        match self.group {
            JointGroup::Antennas => RECORDED_VELOCITY_LIMIT_ANTENNAS,
            JointGroup::Legs | JointGroup::BodyYaw => RECORDED_VELOCITY_LIMIT_HEAD,
        }
    }
}

/// What each class of servo achieved, off the run's own sample stream.
///
/// The measurement a capability run exists to take: not how fast the profile
/// says the joint may move, but how fast it moved while it was chasing
/// something far enough away to be trying. A class whose figures sit under its
/// configured pair is a class whose motor, not the generator, is the binding
/// constraint -- which is the state in which the model the tracking detector
/// screens on stops describing the machine.
///
/// Walked over stretches of consecutive grid cycles: a per-period travel across
/// a slot no sample attended is two periods' motion read as one, and a ramp
/// measured across a gap is not a ramp. Every figure is per period of the run's
/// own grid, which is what [`ClassCapability::velocity_units`] turns into the
/// register's units.
#[must_use]
pub fn capability(samples: &[Logged<PoseSampleWire>], grid: Grid) -> [ClassCapability; 3] {
    let mut ordered: Vec<&Logged<PoseSampleWire>> = samples.iter().collect();
    ordered.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
    let mut tallies: [ClassTally; JointGroup::ALL.len()] = Default::default();
    for stretch in stretches(&ordered, grid) {
        walk_stretch(&stretch, &mut tallies);
    }
    JointGroup::ALL.map(|group| {
        let tally = std::mem::take(&mut tallies[slot(group)]);
        tally.finish(group, grid.period_ns)
    })
}

/// One class's figures as the walk folds them, before they are ranked.
///
/// An accumulator per class rather than one array per figure: the walk pushes
/// four different series and a listing, and five parallel arrays indexed by
/// class is five chances to index one of them with another's slot.
#[derive(Default)]
struct ClassTally {
    /// Per-period travel over every chasing sample, radians.
    travel: Vec<f64>,
    /// The same travels cut by the error the joint stood at, band `n` covering
    /// `[(n + 1) · CHASE_GAP_RAD, (n + 2) · CHASE_GAP_RAD)`.
    binned: Vec<Vec<f64>>,
    /// Every chasing period's gain on the period before it, where it gained at
    /// least an encoder count, radians per period squared.
    increase: Vec<f64>,
    /// Every goal step written to this class, in run order.
    steps: Vec<GoalStep>,
    /// How many chasing samples carried a non-finite reading and were left out
    /// of every figure.
    unreadable: usize,
}

impl ClassTally {
    /// File one chasing sample's travel under the error it was made at.
    ///
    /// The error is finite, which the walk establishes: an infinite one indexes
    /// a band at the top of the address space.
    fn bin(&mut self, error: f64, travel: f64) {
        let band = (error / CHASE_GAP_RAD).floor() as usize;
        // A chase is more than a gap behind by construction, so band 1 is the
        // first; saturating rather than asserting, because a rounding at the
        // boundary must not panic an analyzer.
        let index = band.saturating_sub(1);
        if self.binned.len() <= index {
            self.binned.resize_with(index + 1, Vec::new);
        }
        self.binned[index].push(travel);
    }

    /// The class's figures, every series ranked.
    fn finish(mut self, group: JointGroup, period_ns: i64) -> ClassCapability {
        self.travel.sort_by(f64::total_cmp);
        self.increase.sort_by(f64::total_cmp);
        let bins: Vec<ErrorBin> = self
            .binned
            .iter_mut()
            .enumerate()
            .map(|(index, band)| {
                band.sort_by(f64::total_cmp);
                ErrorBin {
                    low: (index + 1) as f64 * CHASE_GAP_RAD,
                    high: (index + 2) as f64 * CHASE_GAP_RAD,
                    samples: band.len(),
                    travel_p10: percentile(band, 0.10),
                    travel_p50: percentile(band, 0.50),
                    travel_p90: percentile(band, 0.90),
                }
            })
            .collect();
        let (regime, fell_away) = regime_of(&bins);
        // The ramp median is the class's acceleration, so it exists only where
        // the class was written enough steps for a median to be the motor's
        // rather than one motion's.
        let mut ramps: Vec<f64> = self.steps.iter().filter_map(|step| step.ramp).collect();
        ramps.sort_by(f64::total_cmp);
        let read_ramps = self.steps.len() >= CAPABILITY_STEP_MIN_SAMPLES && !ramps.is_empty();
        ClassCapability {
            group,
            chasing: self.travel.len(),
            travel_p10: percentile(&self.travel, 0.10),
            travel_p50: percentile(&self.travel, 0.50),
            travel_p90: percentile(&self.travel, 0.90),
            travel_max: self.travel.last().copied().unwrap_or(0.0),
            bins,
            regime,
            fell_away,
            increases: self.increase.len(),
            increase_p50: percentile(&self.increase, 0.50),
            increase_p90: percentile(&self.increase, 0.90),
            steps: self.steps,
            unreadable: self.unreadable,
            step_ramp_p50: read_ramps.then(|| percentile(&ramps, 0.50)),
            step_ramp_p90: read_ramps.then(|| percentile(&ramps, 0.90)),
            period_ns,
        }
    }
}

/// Whether two bands' median travel is one speed.
///
/// [`CAPABILITY_PLATEAU_RATIO`]'s own meaning, in one place: the two are one
/// speed when neither stands more than the ratio above the other. Symmetric,
/// because which of two bands came out faster is not the question -- and a zero
/// median, whose ratio is not a number, is not one speed with anything.
fn one_speed(a: f64, b: f64) -> bool {
    let ratio = a.max(b) / a.min(b);
    matches!(
        ratio.partial_cmp(&CAPABILITY_PLATEAU_RATIO),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    )
}

/// What the bins say was holding the class back, with the plateau where the
/// answer is the motor, and the top bands the reading was taken under.
///
/// Off the readable bands alone: an unread band is a band the content never
/// held the class in, and a comparison across one would be a comparison against
/// a handful of samples.
///
/// One walk decides both halves, because a band the reading is taken under is
/// exactly a band the plateau cannot start at.
fn regime_of(bins: &[ErrorBin]) -> (Regime, Vec<FellAway>) {
    let readable: Vec<&ErrorBin> = bins.iter().filter(|bin| bin.readable()).collect();
    // From the top down, a band that lost speed against the band under it is
    // set aside. A servo at its ceiling holds a speed as its error grows; it
    // does not lose speed with more error, so such a band is never a ceiling
    // reading -- it is the periods after a large goal step, when the joint is a
    // whole move behind and still in its dead time or its ramp, or a stall. A
    // band that is one speed with the one under it, or faster than it, ends the
    // walk. The condition is the exact complement of the gain-bound test inside
    // "not one speed", so a zero over a zero -- which `one_speed` refuses and
    // `>` does not order -- falls away too.
    let mut fell_away = Vec::new();
    let mut kept = readable.len();
    while kept >= 2 {
        let top = readable[kept - 1];
        let next = readable[kept - 2];
        if one_speed(top.travel_p50, next.travel_p50) || top.travel_p50 > next.travel_p50 {
            break;
        }
        fell_away.push(FellAway {
            low: top.low,
            high: top.high,
            samples: top.samples,
            travel_p50: top.travel_p50,
            under: next.travel_p50,
        });
        kept -= 1;
    }
    let readable = &readable[..kept];
    let (Some(top), Some(next)) = (
        readable.last(),
        readable.len().checked_sub(2).map(|at| readable[at]),
    ) else {
        return (Regime::ContentBound, fell_away);
    };
    // Gain-bound is the one positive test: the fastest band left is not one
    // speed with the band below it *and* is the faster of the two, so its speed
    // was still rising with its error. Everything else is a speed that has
    // stopped rising.
    if !one_speed(top.travel_p50, next.travel_p50) && top.travel_p50 > next.travel_p50 {
        return (Regime::GainBound, fell_away);
    }
    // The plateau: the top band left and every readable band below it for as
    // long as each adjacent pair reads one speed.
    let mut lowest = readable.len() - 1;
    while lowest > 0 && one_speed(readable[lowest].travel_p50, readable[lowest - 1].travel_p50) {
        lowest -= 1;
    }
    let regime = Regime::MotorBound(Plateau {
        low: readable[lowest].low,
        high: top.high,
        bins: readable.len() - lowest,
        travel: readable[lowest..]
            .iter()
            .map(|bin| bin.travel_p50)
            .fold(f64::INFINITY, f64::min),
    });
    (regime, fell_away)
}

/// Which of the three per-class accumulators a group's figures go in.
///
/// Off [`JointGroup::ALL`] rather than the enum's discriminant, so the
/// accumulators and the array the walk returns are indexed by one statement of
/// the order.
fn slot(group: JointGroup) -> usize {
    JointGroup::ALL
        .iter()
        .position(|candidate| *candidate == group)
        .unwrap_or_default()
}

/// One class's recorded p99.9 residual floor: the figures, and the
/// configuration they are a reading of.
///
/// One record per class rather than three parallel arrays, because the three
/// classes' floors are no longer three readings of one overlay — the legs' come
/// off different tours from the other two, and the antennas' come off gains the
/// tree has moved on from. Positional alignment across the classes would say a
/// relationship that is not there, and every reader of a figure needs the
/// configuration beside it: a fresh worst read against a floor measured under
/// some other configuration is a comparison of two machines. Carrying the
/// provenance in the record is what lets the report line print it beside the
/// range instead of leaving it to a reader of this source.
pub struct RecordedFloor {
    /// The three tours' p99.9 residuals, radians, in tour order.
    ///
    /// Three and not a slice: the arity is what makes the range below a range
    /// of figures that exist, so a class that lost its tours cannot fold to a
    /// sentinel. Three and not one because what a candidate pair's p99.9 is
    /// read for is *growth*, and the run-to-run spread between identically
    /// configured tours is the noise floor of that reading.
    pub figures: [f64; 3],
    /// What the figures were read at, as the report prints it after "over three
    /// tours" — so a class whose floor is not its shipping configuration says
    /// so on the bench and not only in this file.
    ///
    /// The tour count is the array's own arity and is not carried twice.
    pub configuration: &'static str,
}

/// The body yaw's p99.9 residual under the shipped `20 / 50` pair, one figure
/// per tour, in tour order.
///
/// Three figures and not one, because what a candidate pair's p99.9 is read for
/// is *growth*, and the run-to-run spread between identically configured tours
/// is the noise floor of that reading: a candidate landing inside this range has
/// not grown, one above its top has, by at least the amount above. A single
/// tour's figure would have every reader mistaking the spread for a change.
///
/// Read over three tours of the whole library at the `20 / 50` pair the class
/// still runs, the `200 / 0 / 0` gains it still runs, and an identical clip
/// library, walked at the measured dead time of two samples; provenance in
/// `docs/servo-tuning.md`. The class was measured gain-bound at that pair — its
/// loop and not its motor is what holds it back — so the pair was left there
/// and these figures are the shipping configuration's.
///
/// The other family of recorded residual figures off those same tours is the
/// worst per joint group, `RECORDED_WORST_HEAD_RESIDUAL_RAD` and
/// `RECORDED_WORST_ANTENNA_RESIDUAL_RAD` in `reachy_motion::tick`, which is
/// what the tracking screen is sized on. Each class's figures are one
/// configuration's reading, at the pair and the gains the tree ships that class
/// at, and are re-baked together for that class or not at all: a fresh worst
/// printed against a noise floor measured under some other configuration is a
/// comparison of two machines. Where a figure is not that class's shipping
/// configuration, its own comment says so — the antennas' record below.
pub const RECORDED_P999_BODY_YAW_RESIDUAL_RAD: RecordedFloor = RecordedFloor {
    figures: [0.2754, 0.3264, 0.2488],
    configuration: "at the pair and gains this class ships",
};

/// The legs' p99.9 residual over three tours at the pair the class is
/// commissioned at, `287 / 326`, in tour order.
///
/// Not the same three tours as the records either side of it: those read the
/// legs under `20 / 50`, which the class no longer runs, and a candidate's
/// growth read against a floor measured under a superseded pair is a comparison
/// of two machines. These three ran the whole library, the same commissioned leg
/// pair, the same leg gains `800 / 100 / 300`, the same clip library and the
/// same commanded leg trajectories, walked at the measured dead time of two
/// samples. What differed between them is the antennas' and the body yaw's
/// pairs, which are other servos and do not enter a leg's residual: the legs
/// read 0.1651 / 0.1674 / 0.1650 rad, a spread of 0.0023 rad against the
/// 0.016 rad the class spread over the three `20 / 50` tours. So they are three
/// identically configured tours *for this class*, and nobody should read them
/// as three tours of the shipping overlay.
///
/// The confirmation tour that commissioned the pair is not folded in: this
/// record is the floor, and that tour is the first reading against it, in
/// `docs/servo-tuning.md`.
pub const RECORDED_P999_LEGS_RESIDUAL_RAD: RecordedFloor = RecordedFloor {
    figures: [0.1651, 0.1674, 0.1650],
    configuration: "at the pair and gains this class ships, on tours of its own",
};

/// The antennas' p99.9 residual over the same three tours as the body yaw's, in
/// the same order.
///
/// Read at the `500 / 0 / 100` gains those tours ran, which this deployment no
/// longer ships: the antennas went to the vendor's `200 / 0 / 0` afterwards,
/// and the proportional term is what sets how far a joint follows behind its
/// generator. So this record is a reading of the pair at gains the machine has
/// moved off, and the first tour figure at the shipping configuration — the
/// confirmation tour's antenna p99.9 — is in `docs/servo-tuning.md` rather than
/// here. Three tours at that configuration are what re-bake it.
pub const RECORDED_P999_ANTENNAS_RESIDUAL_RAD: RecordedFloor = RecordedFloor {
    figures: [0.2999, 0.2759, 0.3177],
    configuration: "at gains this class no longer runs",
};

/// One class's recorded p99.9 residual floor.
fn recorded_p999(group: JointGroup) -> &'static RecordedFloor {
    match group {
        JointGroup::BodyYaw => &RECORDED_P999_BODY_YAW_RESIDUAL_RAD,
        JointGroup::Legs => &RECORDED_P999_LEGS_RESIDUAL_RAD,
        JointGroup::Antennas => &RECORDED_P999_ANTENNAS_RESIDUAL_RAD,
    }
}

/// The low and high of a class's recorded p99.9 residuals, radians.
///
/// The pair, and not the per-tour figures, is what every reader of these
/// figures wants: the range is the noise floor a candidate pair's p99.9 is read
/// against, and low-then-high is the order the report line prints. Stating the
/// reduction once keeps the order a property of this function rather than of
/// each caller, and keeps the tour count out of the access path.
#[must_use]
pub fn recorded_p999_range(group: JointGroup) -> (f64, f64) {
    let [first, second, third] = recorded_p999(group).figures;
    (first.min(second).min(third), first.max(second).max(third))
}

/// What a class's recorded p99.9 range was read at, as the report prints it.
///
/// Beside the range wherever the range is printed: the reader who needs it is
/// the one on the bench comparing a fresh figure with the floor, and for the
/// antennas that comparison is between two configurations.
#[must_use]
pub fn recorded_p999_configuration(group: JointGroup) -> &'static str {
    recorded_p999(group).configuration
}

/// One sample's two nine-row readings and its instant, as a capability walk
/// needs them.
///
/// The instant is carried because the goal-step listing names it: a step's own
/// nanoseconds are what a person hands `//cogs:trace_export` to cut the periods
/// after it out of the log and read the ramp for themselves.
struct Step {
    /// The sample's nominal instant, nanoseconds.
    at_ns: i64,
    /// The nine angles it was read at.
    present: [f64; ROWS.len()],
    /// The nine angles the driver was holding when it was read.
    commanded: [f64; ROWS.len()],
}

/// The run's samples cut into stretches of consecutive grid cycles, each
/// sample carrying both a reading and the setpoint it was held under.
///
/// A sample missing either ends the stretch: the travel between two readings a
/// slot apart is not a per-period travel, and a chasing test needs the setpoint
/// that was standing a dead time before the reading.
fn stretches(ordered: &[&Logged<PoseSampleWire>], grid: Grid) -> Vec<Vec<Step>> {
    let mut out: Vec<Vec<Step>> = Vec::new();
    let mut current: Vec<Step> = Vec::new();
    let mut previous: Option<i64> = None;
    for sample in ordered {
        let at_ns = sample.message.nominal_time().as_nanos();
        let (cycle, _) = grid.at(at_ns);
        let consecutive = previous.is_some_and(|before| cycle == before + 1);
        let step = present_rows(&sample.message)
            .zip(commanded_rows(&sample.message))
            .map(|(present, commanded)| Step {
                at_ns,
                present,
                commanded,
            });
        match step {
            Some(step) => {
                if !consecutive && !current.is_empty() {
                    out.push(std::mem::take(&mut current));
                }
                current.push(step);
                previous = Some(cycle);
            }
            None => {
                if !current.is_empty() {
                    out.push(std::mem::take(&mut current));
                }
                previous = None;
            }
        }
    }
    if !current.is_empty() {
        out.push(current);
    }
    out
}

/// Every chasing sample and every goal step in one stretch, folded into the
/// per-class figures.
fn walk_stretch(stretch: &[Step], tallies: &mut [ClassTally; JointGroup::ALL.len()]) {
    let travel = |at: usize, index: usize| {
        (stretch[at].present[index] - stretch[at - 1].present[index]).abs()
    };
    // Whether the joint was chasing at this sample: every setpoint the driver
    // held over the dead time -- which is every goal the servo could have been
    // answering -- stood further than the gap from where the joint then was. All
    // of them and not the oldest alone, so a joint that has arrived at any of
    // them is not counted as chasing the one it has left behind.
    //
    // A dead time is at least one period, so every sample a chase can be read
    // at has a sample before it for the travel to be measured against.
    const _: () = assert!(RESPONSE_DEAD_SAMPLES >= 1);
    let chasing = |at: usize, index: usize| {
        at >= RESPONSE_DEAD_SAMPLES
            && (1..=RESPONSE_DEAD_SAMPLES).all(|back| {
                (stretch[at - back].commanded[index] - stretch[at - 1].present[index]).abs()
                    > CHASE_GAP_RAD
            })
    };
    for at in RESPONSE_DEAD_SAMPLES..stretch.len() {
        for joint in ROWS {
            let (Some(index), Some(group)) = (row(joint), group_of(joint)) else {
                continue;
            };
            if !chasing(at, index) {
                continue;
            }
            let moved = travel(at, index);
            let tally = &mut tallies[slot(group)];
            // The error the travel was made at: how far the goal the driver was
            // holding stood from the joint at the sample before, which is the
            // error the loop was answering when it made this period's move.
            let error = (stretch[at - 1].commanded[index] - stretch[at - 1].present[index]).abs();
            // A non-finite reading is corruption in the log and not a fast
            // joint: it stands further than the gap from anything, so it
            // passes the chase test, and the band it indexes is the top of the
            // address space. Counted and left out of every figure, so a run
            // holding one is still a run the rest of the report is about.
            if !moved.is_finite() || !error.is_finite() {
                tally.unreadable += 1;
                continue;
            }
            tally.travel.push(moved);
            tally.bin(error, moved);
            // The increase: a chasing period that travelled at least an encoder
            // count further than the one before it was still accelerating, and
            // the gain is what it accelerated at. A note about the class, and
            // the acceleration of a class whose speed never stopped rising --
            // where a motor-bound class's own acceleration is read off its goal
            // steps below.
            if at >= 2 {
                let gained = moved - travel(at - 1, index);
                if gained >= COUNT_RAD {
                    tally.increase.push(gained);
                }
            }
        }
    }
    // The goal steps, over the whole stretch and not only its chasing samples:
    // a step is read at the period the goal moved in, which is a period the
    // joint had not yet answered and so was not chasing in.
    for at in 1..stretch.len() {
        for joint in ROWS {
            let (Some(index), Some(group)) = (row(joint), group_of(joint)) else {
                continue;
            };
            let step = stretch[at].commanded[index] - stretch[at - 1].commanded[index];
            // A step is a goal moved further than a chase gap in one period
            // while the joint itself stood still. Both halves matter: a smaller
            // step does not put the joint into a chase, and a step written to a
            // joint already moving ramps from the speed it had rather than from
            // rest.
            if step.abs() <= CHASE_GAP_RAD || travel(at, index) >= COUNT_RAD {
                continue;
            }
            let periods: Vec<f64> = (1..=STEP_LISTING_PERIODS)
                .map_while(|ahead| (at + ahead < stretch.len()).then(|| travel(at + ahead, index)))
                .collect();
            // The ramp is the second post-write period's travel less the
            // first's: at this dead time the joint starts late in the first and
            // the second is its first whole period of acceleration.
            let ramp = match periods.as_slice() {
                [first, second, ..] => Some(second - first),
                _ => None,
            };
            tallies[slot(group)].steps.push(GoalStep {
                joint,
                at_ns: stretch[at].at_ns,
                step,
                travel: periods,
                ramp,
            });
        }
    }
}

/// What each class achieved, printed against the limits its servos were
/// recorded at.
///
/// Notes and never verdicts: what a class's figures mean for the pair it should
/// be commissioned at is a decision made by a person reading the run, against
/// the class's own load and the content the run played. What the report owes
/// that reading is the figures and the limit they sit under.
///
/// TODO(capability-report-volume): every band and every goal step is printed,
/// on every run, which is on the order of a hundred lines per class on a
/// library tour.
pub fn capabilities(measured: &[ClassCapability; 3], report: &mut Report) {
    for class in measured {
        // Before the figures, because a class whose samples were all
        // unreadable has none: a log carrying a non-finite angle is a log to
        // ask about, and a report that measured nothing over it has to say
        // which of the two happened.
        if class.unreadable > 0 {
            report.note(format!(
                "capability {}: {} chasing sample(s) carried a non-finite reading and measured \
                 nothing; a log holding one is a log to ask about",
                class.group.name(),
                class.unreadable,
            ));
        }
        if class.chasing == 0 {
            report.note(format!(
                "capability {}: no sample found this class chasing a setpoint more than \
                 {CHASE_GAP_RAD} rad away, so the run says nothing about what its motors can do",
                class.group.name()
            ));
            continue;
        }
        report.note(format!(
            "capability {}: {} chasing sample(s), travel per period p10 {:.5} rad ({:.0} units), \
             p50 {:.5} ({:.0}), p90 {:.5} ({:.0}), max {:.5} ({:.0}), against a recorded Velocity \
             Limit of {} units",
            class.group.name(),
            class.chasing,
            class.travel_p10,
            class.velocity_units(class.travel_p10),
            class.travel_p50,
            class.velocity_units(class.travel_p50),
            class.travel_p90,
            class.velocity_units(class.travel_p90),
            class.travel_max,
            class.velocity_units(class.travel_max),
            class.recorded_velocity_limit(),
        ));
        let recorded = recorded_capability(class.group);
        report.note(format!(
            "capability {}: the recorded reading of this class is acceleration {} / velocity {} \
             units, off the kept capability tour",
            class.group.name(),
            recorded.acceleration,
            recorded.velocity,
        ));
        travel_against_error(class, report);
        report.note(format!(
            "capability {}: {} chasing period(s) travelled at least a count further than the \
             period before, increase p50 {:.6} rad/period² ({:.0} units), p90 {:.6} ({:.0})",
            class.group.name(),
            class.increases,
            class.increase_p50,
            class.acceleration_units(class.increase_p50),
            class.increase_p90,
            class.acceleration_units(class.increase_p90),
        ));
        goal_steps(class, report);
    }
}

/// One class's travel cut by the error it was made at, and the regime the cut
/// reads as.
///
/// Every band, read or not: a band the content never held the class in is a
/// reading about the content, and a table that printed only the readable bands
/// would leave a person unable to tell a class the library never pushed from a
/// class whose bands are all full.
fn travel_against_error(class: &ClassCapability, report: &mut Report) {
    for bin in &class.bins {
        if bin.readable() {
            report.note(format!(
                "capability {}: error {:.2}–{:.2} rad, n {}, travel p10 {:.0} units, p50 {:.0}, \
                 p90 {:.0}",
                class.group.name(),
                bin.low,
                bin.high,
                bin.samples,
                class.velocity_units(bin.travel_p10),
                class.velocity_units(bin.travel_p50),
                class.velocity_units(bin.travel_p90),
            ));
        } else {
            report.note(format!(
                "capability {}: error {:.2}–{:.2} rad, n {}, unread -- under \
                 {CAPABILITY_BIN_MIN_SAMPLES} samples",
                class.group.name(),
                bin.low,
                bin.high,
                bin.samples,
            ));
        }
    }
    for band in &class.fell_away {
        let against = match band.ratio() {
            Some(ratio) => format!("{ratio:.2} times slower"),
            None => "read zero".to_string(),
        };
        report.note(format!(
            "capability {}: error {:.2}–{:.2} rad, n {}, travel p50 {:.0} units -- fell away from \
             the band under it ({:.0} units, {against}), read past as the periods after a goal \
             step or a stall",
            class.group.name(),
            band.low,
            band.high,
            band.samples,
            class.velocity_units(band.travel_p50),
            class.velocity_units(band.under),
        ));
    }
    let name = class.regime.name();
    let read_under = if class.fell_away.is_empty() {
        String::new()
    } else {
        format!(
            ", read under {} band(s) that fell away",
            class.fell_away.len()
        )
    };
    match class.regime {
        Regime::MotorBound(plateau) => report.note(format!(
            "capability {}: {name} -- the speed stopped rising with the error, plateau \
             {:.2}–{:.2} rad over {} band(s), slowest median in it {:.0} units{read_under}",
            class.group.name(),
            plateau.low,
            plateau.high,
            plateau.bins,
            class.velocity_units(plateau.travel),
        )),
        Regime::GainBound => report.note(format!(
            "capability {}: {name} -- the fastest band left is over {CAPABILITY_PLATEAU_RATIO} \
             times the band below it, so the speed was still rising with the error and the motor \
             was never reached{read_under}",
            class.group.name(),
        )),
        // Content-bound after a fall says nothing about the class, and says so
        // in its own words: the candidate rule for a content-bound class
        // extrapolates above everything observed on the premise that the
        // library never held the class far behind, and a class whose far bands
        // were read past was held exactly there.
        Regime::ContentBound if !class.fell_away.is_empty() => report.note(format!(
            "capability {}: {name} -- the reading needs two speeds and {} band(s) above were read \
             past, so this run is not a measurement of this class and offers no candidate",
            class.group.name(),
            class.fell_away.len(),
        )),
        Regime::ContentBound => report.note(format!(
            "capability {}: {name} -- fewer than two readable error bands, so nothing here says \
             whether this class was ever the binding constraint",
            class.group.name(),
        )),
    }
}

/// Every goal step one class was written, and what the joint did after each.
///
/// A listing and not a statistic, printed step by step: it is the reading the
/// dead time is checked against -- the periods a joint stands still after the
/// write are visible in the row -- and the cross-check on the increase figure
/// above it. The median under it is the class's acceleration, and it exists
/// only where the class was written enough steps for a median to describe the
/// motor.
fn goal_steps(class: &ClassCapability, report: &mut Report) {
    for step in &class.steps {
        let travel: Vec<String> = step
            .travel
            .iter()
            .map(|moved| format!("{:.0}", class.velocity_units(*moved)))
            .collect();
        let ramp = match step.ramp {
            Some(ramp) => format!("{:.0} units", class.acceleration_units(ramp)),
            None => "no ramp -- the stretch ended inside the step".to_string(),
        };
        report.note(format!(
            "capability {}: goal step at {} on {}, {:+.4} rad, travel per period after it {} \
             units, ramp {ramp}",
            class.group.name(),
            step.at_ns,
            Name(step.joint),
            step.step,
            travel.join(", "),
        ));
    }
    match (class.step_ramp_p50, class.step_ramp_p90) {
        (Some(p50), Some(p90)) => report.note(format!(
            "capability {}: {} goal step(s), ramp p50 {:.6} rad/period² ({:.0} units), p90 \
             {:.6} ({:.0})",
            class.group.name(),
            class.steps.len(),
            p50,
            class.acceleration_units(p50),
            p90,
            class.acceleration_units(p90),
        )),
        _ => report.note(format!(
            "capability {}: {} goal step(s), under {CAPABILITY_STEP_MIN_SAMPLES}, so this class \
             has no ramp median and no acceleration figure the steps measured",
            class.group.name(),
            class.steps.len(),
        )),
    }
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

    use reachy_motion::plant::{
        GroupPlants, GroupProfiles, MAX_GAP_PERIODS, Predicted, RESPONSE_DEAD_SAMPLES,
    };

    use brenn_reachy__driver__health_clk_rs::HealthReportWire;
    use reachy_motion::joints::JointGroup;
    use reachy_motion::plant::{
        PROFILE_ACCELERATION_MAX, PlantModel, ProfilePair, SHIPPED_PROFILES,
    };
    use reachy_motion::stillness::COUNT_RAD;

    use super::{
        CAPABILITY_BIN_MIN_SAMPLES, CAPABILITY_PLATEAU_RATIO, CAPABILITY_STEP_MIN_SAMPLES,
        CHASE_GAP_RAD, CONFIG_FILES, ClassCapability, ErrorBin, Grid, RECORDED_CAPABILITY_ANTENNAS,
        RECORDED_CAPABILITY_BODY_YAW, RECORDED_CAPABILITY_LEGS,
        RECORDED_P999_ANTENNAS_RESIDUAL_RAD, RECORDED_P999_BODY_YAW_RESIDUAL_RAD,
        RECORDED_P999_LEGS_RESIDUAL_RAD, RECORDED_VELOCITY_LIMIT_ANTENNAS,
        RECORDED_VELOCITY_LIMIT_HEAD, Regime, Residual, RunConfig, Skips, TEMPERATURE_STOP_C,
        capabilities, capability, health_summary, lags, no_faults, one_speed, percentile,
        recorded_capability, regime_of, residual_stream, residuals, slot, travel_against_error,
    };

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
    fn chase(plant: &GroupPlants, commanded: &[f64], shift: usize) -> Vec<Logged<PoseSampleWire>> {
        let index = driven();
        let model = plant.for_row(index);
        let mut predicted = Predicted::default();
        let mut out = Vec::new();
        for (n, held) in commanded.iter().enumerate() {
            if n >= RESPONSE_DEAD_SAMPLES {
                model.step(&mut predicted, commanded[n - RESPONSE_DEAD_SAMPLES + shift]);
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
    /// does, both as disagreements: how far off the model, whichever side.
    fn worsts(stream: &[(i64, [Residual; ROW_COUNT])]) -> (f64, f64) {
        let index = driven();
        let mut driven_row = 0.0_f64;
        let mut elsewhere = 0.0_f64;
        for (_, residual) in stream {
            for (at, figure) in residual.iter().enumerate() {
                if at == index {
                    driven_row = driven_row.max(figure.magnitude());
                } else {
                    elsewhere = elsewhere.max(figure.magnitude());
                }
            }
        }
        (driven_row, elsewhere)
    }

    /// The residual a sample carries, as a disagreement.
    fn at_cycle(stream: &[(i64, [Residual; ROW_COUNT])], n: i64) -> f64 {
        let index = driven();
        stream
            .iter()
            .find(|(nominal, _)| *nominal == ORIGIN_NS + n * PERIOD_NS)
            .map(|(_, residual)| residual[index].magnitude())
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
        let plant = GroupPlants::default();
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
    /// model is measured at rather than about any delay at all.
    #[test]
    fn a_reading_that_answers_a_period_early_reads_a_residual() {
        let plant = GroupPlants::default();
        let commanded = saturated(60);
        let stream = residual_stream(&chase(&plant, &commanded, 1), grid(), &plant);
        let (driven_row, _) = worsts(&stream);
        let v_max = plant.for_row(driven()).v_max;
        assert!(
            driven_row > 0.5 * v_max,
            "a period of a saturated move is {v_max} rad, and the residual is {driven_row}"
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
        let plant = GroupPlants::default();
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
        let plant = GroupPlants::default();
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
        let plant = GroupPlants::default();
        let mut samples = frozen(60, 2.0);
        let index = driven();
        assert!(
            residual_stream(&samples, grid(), &plant)
                .iter()
                .any(|(_, residual)| residual[index].magnitude() > 0.1),
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
        let plant = GroupPlants::default();
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

    /// The residual line says which side of its own trajectory the joint stood,
    /// because the two sides mean opposite things for a candidate pair.
    ///
    /// Two streams of the same motion: one whose reading answers a period late,
    /// so the joint trails its prediction, and one whose reading answers a
    /// period early, so it leads it. A summary that only carried the
    /// disagreement would print the same figure for both, and the confirmation
    /// run's decision tree would send a machine that outran its model to the
    /// branch that slows it down further.
    #[test]
    fn the_residual_summary_says_which_side_of_the_model_the_joint_stood() {
        let plant = GroupPlants::default();
        let lagging = frozen(60, 2.0);
        let leading = chase(&plant, &saturated(60), 1);
        let index = driven();
        // The two figures the line prints, taken off the same stream the
        // summary reads, and the line itself, so the reading and its wording
        // are pinned together.
        let read = |samples: &[Logged<PoseSampleWire>]| {
            let stream = residual_stream(samples, grid(), &plant);
            let mut behind = 0.0_f64;
            let mut ahead = 0.0_f64;
            for (_, residual) in &stream {
                if residual[index].is_behind() {
                    behind = behind.max(residual[index].magnitude());
                } else if residual[index].is_ahead() {
                    ahead = ahead.max(residual[index].magnitude());
                }
            }
            let mut report = Report::default();
            residuals(&stream, samples, &plant, &mut report);
            let wanted = format!(
                "body yaw: worst {behind:.4} rad behind its own trajectory, worst {ahead:.4} rad \
                 ahead of it"
            );
            assert!(
                report.measured.contains(&wanted),
                "{wanted:?} is not in {:?}",
                report.measured
            );
            (behind, ahead)
        };
        let (behind, ahead) = read(&lagging);
        assert!(
            behind > 0.1 && ahead == 0.0,
            "a joint that never moved under a far goal is only ever behind: {behind} / {ahead}"
        );
        let (behind, ahead) = read(&leading);
        assert!(
            ahead > 5.0 * behind,
            "a reading that answers a period early leads its model: {behind} / {ahead}"
        );
    }

    /// Every class's residual line carries that class's own recorded p99.9
    /// range, low figure first, and the configuration that range was read at.
    ///
    /// The floor is at the point of use because the reading it supports is a
    /// comparison: a candidate pair's p99.9 against what the class's own
    /// recorded configuration did over three tours. A line carrying another
    /// class's range, or the range the wrong way round, would read as growth
    /// that is not there — and so would a line calling the antennas' floor a
    /// shipping figure, which is the one class whose floor was read at gains
    /// the tree has moved off.
    ///
    /// The figures are written out here rather than read back off the
    /// constants: they were transcribed by hand from an offline measurement,
    /// and an expectation computed from the same array cannot see a mistyped
    /// digit or a class wired to another class's figures.
    #[test]
    fn each_class_line_carries_its_own_recorded_p999_range() {
        let plant = GroupPlants::default();
        let samples = frozen(40, 2.0);
        let stream = residual_stream(&samples, grid(), &plant);
        let mut report = Report::default();
        residuals(&stream, &samples, &plant, &mut report);

        // Every transcribed figure, in tour order, and not only the two the
        // range prints: a mistyped middle figure is inside the range it does
        // not move, so the printed line cannot see it.
        assert_eq!(
            RECORDED_P999_BODY_YAW_RESIDUAL_RAD.figures,
            [0.2754, 0.3264, 0.2488]
        );
        assert_eq!(
            RECORDED_P999_LEGS_RESIDUAL_RAD.figures,
            [0.1651, 0.1674, 0.1650]
        );
        assert_eq!(
            RECORDED_P999_ANTENNAS_RESIDUAL_RAD.figures,
            [0.2999, 0.2759, 0.3177]
        );

        // The provenance clause per class, written out the same way: the
        // antennas' floor is the one that is not a shipping reading, and a line
        // that called it one is the misreading the record exists to stop.
        let ranges = [
            (
                JointGroup::BodyYaw,
                "0.2488–0.3264",
                "at the pair and gains this class ships",
            ),
            (
                JointGroup::Legs,
                "0.1650–0.1674",
                "at the pair and gains this class ships, on tours of its own",
            ),
            (
                JointGroup::Antennas,
                "0.2759–0.3177",
                "at gains this class no longer runs",
            ),
        ];
        for (group, range, configuration) in ranges {
            let wanted = format!("recorded p99.9 {range} rad over three tours {configuration},");
            let line = report
                .measured
                .iter()
                .find(|line| line.starts_with(&format!("{}: ", group.name())))
                .unwrap_or_else(|| panic!("a line for {}", group.name()));
            assert!(line.contains(&wanted), "{line}");
        }
    }

    /// The scratch directory this test binary owns, which the harness makes
    /// and empties.
    ///
    /// `TEST_TMPDIR` under bazel -- private per test target and per run, so two
    /// checkouts, a sharded CI run or a second person on the machine cannot
    /// collide on a fixture, and nothing has to be cleaned up on the panic path
    /// where a case has just failed. The system temporary directory is the
    /// fallback for a run outside the harness.
    fn scratch() -> std::path::PathBuf {
        std::env::var_os("TEST_TMPDIR").map_or_else(std::env::temp_dir, std::path::PathBuf::from)
    }

    /// A configuration directory as a run carries it home, with each file's
    /// text as given.
    fn staged(name: &str, texts: [&str; CONFIG_FILES.len()]) -> std::path::PathBuf {
        let log = scratch().join(format!("run-{name}"));
        let root = log.join("config").join("cogs");
        std::fs::create_dir_all(&root).expect("a temporary directory");
        for (text, file) in texts.iter().zip(CONFIG_FILES) {
            let leaf = file.rsplit('/').next().expect("a file name");
            std::fs::write(root.join(leaf), text).expect("a temporary file");
        }
        log
    }

    /// The three files as this tree ships them, which is what a staged run is
    /// built out of when the case is about something else.
    fn tree_texts() -> [String; CONFIG_FILES.len()] {
        CONFIG_FILES.map(|file| std::fs::read_to_string(file).expect("this tree's own copy"))
    }

    /// Every figure is read from the fields of its file and never from the
    /// prose around them, and a file that states no value is a failure rather
    /// than a default.
    ///
    /// The comment case is the one that would be silent: `servo_profile`'s own
    /// comment block discusses the bench's pair, so a scan that took the first
    /// match anywhere would judge a log under a pair nobody commissioned. The
    /// classes are read in whatever order the file states them, and each name
    /// carries its own class: a scan that matched a bare `profile_velocity`
    /// would hand every class the same number.
    #[test]
    fn a_configuration_is_read_from_its_fields_and_not_from_the_comments_around_them() {
        let gains = "legs_p: 800\nlegs_i: 100\nlegs_d: 300\n\
                     body_yaw_p: 200\nbody_yaw_i: 0\nbody_yaw_d: 0\n\
                     antennas_p: 200\nantennas_i: 0\nantennas_d: 0\n";
        let commented = staged(
            "commented",
            [
                "# the bench ran legs_profile_velocity: 600\n\
                 # and legs_profile_acceleration: 400\n\
                 antennas_profile_velocity: 70\n\
                 antennas_profile_acceleration: 40\n\
                 legs_profile_acceleration: 20\nlegs_profile_velocity: 50\n\
                 body_yaw_profile_acceleration: 30\nbody_yaw_profile_velocity: 60\n",
                gains,
                "# tracking_armed: false, in the prose\ntracking_armed: true\n",
            ],
        );
        let config = RunConfig::read(&commented).expect("a configuration");
        assert_eq!(
            config.profiles,
            GroupProfiles {
                legs: ProfilePair {
                    acceleration: 20,
                    velocity: 50,
                },
                yaw: ProfilePair {
                    acceleration: 30,
                    velocity: 60,
                },
                antennas: ProfilePair {
                    acceleration: 40,
                    velocity: 70,
                },
            }
        );
        assert_eq!(config.gains, reachy_motion::arm::DEFAULT_GAINS);
        assert!(config.tracking_armed);

        let mut texts = tree_texts();
        texts[0] = "legs_profile_acceleration: 20\nlegs_profile_velocity: 50\n\
                    body_yaw_profile_acceleration: 30\nbody_yaw_profile_velocity: 60\n\
                    antennas_profile_acceleration: 40\n"
            .to_string();
        let partial = staged("partial", texts.each_ref().map(String::as_str));
        assert!(
            RunConfig::read(&partial)
                .is_err_and(|says| says.contains("states no antennas_profile_velocity")),
            "a pair with one half missing is named, not defaulted"
        );

        let mut texts = tree_texts();
        texts[1] = "legs_p: 800\nlegs_i: 100\nlegs_d: 300\n\
                    body_yaw_p: 200\nbody_yaw_i: 0\nbody_yaw_d: 0\n\
                    antennas_p: quick\nantennas_i: 0\nantennas_d: 100\n"
            .to_string();
        let unreadable = staged("unreadable", texts.each_ref().map(String::as_str));
        assert!(
            RunConfig::read(&unreadable)
                .is_err_and(|says| says.contains("antennas_p is no gain value")),
            "a gain that is not a number is named"
        );

        let mut texts = tree_texts();
        texts[2] = "tracking_armed: maybe\n".to_string();
        let neither = staged("neither", texts.each_ref().map(String::as_str));
        assert!(
            RunConfig::read(&neither).is_err_and(|says| says.contains("neither true nor false")),
            "an arming that is neither is a refusal, not an assumption"
        );

        // Everything protobuf's text format takes, because it is what read
        // these files on the unit: a spelling the machine ran and this scan
        // refused would cost the run rather than the file.
        let spelled = staged(
            "spelled",
            [
                "legs_profile_acceleration: 0x14  # twenty\nlegs_profile_velocity: 50\n\
                 body_yaw_profile_acceleration: 30\nbody_yaw_profile_velocity: 60\n\
                 antennas_profile_acceleration: 40\nantennas_profile_velocity: 70\n",
                gains,
                "tracking_armed: True # the campaign's own runs\n",
            ],
        );
        let config = RunConfig::read(&spelled).expect("a configuration the loader would take");
        assert_eq!(
            config.profiles.legs,
            ProfilePair {
                acceleration: 20,
                velocity: 50
            },
            "0x14 is twenty"
        );
        assert!(config.tracking_armed, "True is protobuf's true");

        let mut texts = tree_texts();
        texts[2] = "tracking_armed: 0\n".to_string();
        let numeric = staged("numeric", texts.each_ref().map(String::as_str));
        assert!(
            !RunConfig::read(&numeric)
                .expect("a configuration")
                .tracking_armed,
            "and 0 is protobuf's false, which is a disarmed run"
        );
    }

    /// A run whose records carry no configuration is refused: what a machine
    /// was commissioned with is not something an analyzing host can supply, and
    /// a report that judged residuals under an assumed pair would be a verdict
    /// about a machine nobody ran.
    #[test]
    fn a_run_that_carries_no_configuration_is_refused_by_name() {
        let bare = scratch().join("run-bare");
        std::fs::create_dir_all(&bare).expect("a temporary directory");
        let _ = std::fs::remove_dir_all(bare.join("config"));
        assert!(
            RunConfig::read(&bare)
                .is_err_and(|says| says.contains("no configuration beside these records")),
            "the refusal says what is missing"
        );
    }

    /// What a report says about the configuration: the three files' figures,
    /// whether the run's copies are the tree's, and the one verdict — a
    /// disarmed detector is a failed run, with nothing to excuse it.
    #[test]
    fn the_configuration_notes_say_what_ran_and_whether_it_was_the_trees() {
        let tree = tree_texts();
        let shipped = staged("shipped", tree.each_ref().map(String::as_str));
        let mut report = Report::default();
        RunConfig::read(&shipped)
            .expect("a configuration")
            .configuration(&mut report);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("configuration cogs/servo_profile.textproto")),
            "{:?}",
            report.measured
        );
        assert!(
            report.measured.iter().any(|line| line.contains(
                "configuration cogs/mover_params.textproto: \
                                           tracking_armed: true"
            )),
            "{:?}",
            report.measured
        );
        assert!(
            !report
                .measured
                .iter()
                .any(|line| line.contains("is not the tree's")),
            "a run on the tree's own files says nothing about an overlay: {:?}",
            report.measured
        );

        let mut texts = tree_texts();
        texts[0] = "legs_profile_acceleration: 32767\nlegs_profile_velocity: 445\n\
                    body_yaw_profile_acceleration: 32767\nbody_yaw_profile_velocity: 445\n\
                    antennas_profile_acceleration: 32767\nantennas_profile_velocity: 1620\n"
            .to_string();
        texts[2] = "tracking_armed: false\n".to_string();
        let overlaid = staged("overlaid", texts.each_ref().map(String::as_str));
        let mut report = Report::default();
        RunConfig::read(&overlaid)
            .expect("a configuration")
            .configuration(&mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line
                    .contains("this run's cogs/servo_profile.textproto is not the tree's")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains("tracking detector was disarmed")),
            "a disarmed run never reads green: {:?}",
            report.findings
        );
    }

    /// One health reading, at cycle `n`.
    fn reading(n: i64, id: u8, temp_c: i8, volts: f64, bits: u8) -> Logged<HealthReportWire> {
        let mut message = HealthReportWire::new();
        message.set_id(id);
        message.set_temp_c(temp_c);
        message.set_volts(volts);
        message.set_bits(bits);
        message.set_sample_time(SyncTime::from_nanos(ORIGIN_NS + n * PERIOD_NS));
        Logged {
            at_ns: ORIGIN_NS + n * PERIOD_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// The temperature a servo peaked at, and when, survives a rotation that
    /// read it cooler afterwards.
    ///
    /// The reading the campaign wants is the hottest the run got, which the last
    /// reading alone does not carry: a tour that heats a servo and then plays
    /// something slow would report the cooling.
    #[test]
    fn the_health_summary_carries_the_peak_and_not_just_the_last_reading() {
        let readings = vec![
            reading(0, 10, 31, 7.4, 0),
            reading(5, 10, 48, 7.1, 0),
            reading(9, 10, 44, 7.6, 0),
        ];
        let mut report = Report::default();
        health_summary(&readings, &mut report);
        let line = report
            .measured
            .iter()
            .find(|line| line.contains("servo 10:"))
            .expect("a line per servo")
            .clone();
        assert!(line.contains("7.10 V to 7.60 V"), "{line}");
        assert!(
            line.contains("31 C at the first reading and 44 C at the last"),
            "{line}"
        );
        // Seconds into the run and not an epoch nanosecond: the peak is read
        // against what the machine was playing when it was reached.
        assert!(line.contains("peak 48 C at 0.1 s into the run"), "{line}");
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A servo that reached the stop temperature fails the run, and one that
    /// stopped a degree short of it does not.
    ///
    /// The rule is `>=`: the constant is the reading at which a run stops being
    /// healthy, not the first reading past it.
    #[test]
    fn a_servo_at_the_stop_temperature_fails_the_run_and_one_below_it_does_not() {
        let mut report = Report::default();
        health_summary(
            &[
                reading(0, 13, 31, 7.4, 0),
                reading(4, 13, TEMPERATURE_STOP_C, 7.4, 0),
                reading(9, 13, 35, 7.4, 0),
            ],
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains(&format!("servo 13 ({TEMPERATURE_STOP_C} C at 0.1 s)"))),
            "the peak names the servo and when it was reached: {:?}",
            report.findings
        );

        let mut cooler = Report::default();
        health_summary(
            &[
                reading(0, 13, 31, 7.4, 0),
                reading(4, 13, TEMPERATURE_STOP_C - 1, 7.4, 0),
            ],
            &mut cooler,
        );
        assert!(cooler.findings.is_empty(), "{:?}", cooler.findings);
    }

    /// Two servos over the ceiling are one finding naming both, with the count
    /// the set's size.
    ///
    /// The shape the error-bit rule uses, and for its reason: a finding per hot
    /// servo on a run that cooked the whole head would bury every other verdict
    /// the report carries.
    #[test]
    fn every_servo_over_the_ceiling_lands_in_one_finding_with_the_count() {
        let mut report = Report::default();
        health_summary(
            &[
                reading(0, 13, 31, 7.4, 0),
                reading(0, 14, 31, 7.4, 0),
                reading(0, 15, 31, 7.4, 0),
                reading(4, 13, TEMPERATURE_STOP_C, 7.4, 0),
                reading(8, 15, TEMPERATURE_STOP_C + 2, 7.4, 0),
            ],
            &mut report,
        );
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        let line = &report.findings[0];
        assert!(
            line.contains(&format!(
                "2 of the servos the health rotation read reached {TEMPERATURE_STOP_C} C"
            )),
            "{line}"
        );
        assert!(
            line.contains(&format!("servo 13 ({TEMPERATURE_STOP_C} C at 0.1 s)")),
            "{line}"
        );
        assert!(
            line.contains(&format!("servo 15 ({} C at 0.2 s)", TEMPERATURE_STOP_C + 2)),
            "{line}"
        );
        // The servo that stayed cool is named nowhere in the verdict.
        assert!(!line.contains("servo 14"), "{line}");
    }

    /// A bit latched in the middle of a run is a finding even where the servo
    /// read clean afterwards, and the voltage bit alone is not.
    #[test]
    fn a_bit_latched_mid_run_is_a_finding_and_the_voltage_bit_alone_is_not() {
        let readings = vec![
            reading(0, 11, 30, 7.4, 0x01),
            reading(3, 11, 30, 7.4, 0x21),
            reading(6, 11, 30, 7.4, 0x01),
            reading(6, 12, 30, 7.4, 0x01),
        ];
        let mut report = Report::default();
        health_summary(&readings, &mut report);
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains("servo 11 (0x21)")),
            "the worst byte is the run's, not the last reading's: {:?}",
            report.findings
        );
        assert!(
            !report.findings.iter().any(|line| line.contains("servo 12")),
            "the input-voltage bit alone is this machine's arithmetic: {:?}",
            report.findings
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("the input-voltage bit is latched on 1 of the servos")),
            "{:?}",
            report.measured
        );
    }

    /// A rotation that reported nothing says so, rather than printing a machine
    /// of no servos.
    #[test]
    fn a_run_with_no_health_reading_says_so() {
        let mut report = Report::default();
        health_summary(&[], &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("the health rotation reported nothing")),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A run in which one antenna chases a far setpoint under its own
    /// generator, from `at` on: every other joint stands still.
    ///
    /// The setpoint jumps while the joint is still at rest, and the reading
    /// follows a dead time later, which is what a real move looks like -- and
    /// is what puts a ramp where the walk can see one.
    fn far_chase(periods: i64, goal: f64, model: PlantModel) -> Vec<Logged<PoseSampleWire>> {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        let mut state = Predicted::default();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        for n in 0..periods {
            samples.push(sample(n, &present, &commanded));
            // The setpoint the driver holds from the third period on, and the
            // reading that answers it a dead time after that.
            if n >= 2 {
                commanded[index] = goal;
            }
            if n >= 2 + RESPONSE_DEAD_SAMPLES as i64 {
                model.step(&mut state, goal);
                present[index] = state.position;
            }
        }
        samples
    }

    /// A run of far chases on one antenna, each opened by a goal step of
    /// `reach` radians and answered by `model` a dead time later.
    ///
    /// The shape a capability tour has and `far_chase` does not: several goal
    /// steps, so the step listing has a median under it and the error bands
    /// fill. The goal accumulates in one direction, so no move starts before
    /// the last has arrived at a goal it then leaves behind.
    fn stepped_chases(
        model: PlantModel,
        moves: usize,
        reach: f64,
        hold: usize,
    ) -> Vec<Logged<PoseSampleWire>> {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        let mut state = Predicted::default();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        let mut goals: Vec<f64> = Vec::new();
        for n in 0..2 + moves * hold {
            samples.push(sample(n as i64, &present, &commanded));
            if n >= 2 && (n - 2) % hold == 0 {
                commanded[index] += reach;
            }
            goals.push(commanded[index]);
            if n >= RESPONSE_DEAD_SAMPLES {
                model.step(&mut state, goals[n - RESPONSE_DEAD_SAMPLES]);
                present[index] = state.position;
            }
        }
        samples
    }

    /// A joint running its own generator's trapezoid at a velocity its
    /// acceleration reaches in one period reads back as that generator: every
    /// readable error band's median travel is the profile velocity, the class
    /// reads motor-bound, and the plateau's own figure is the velocity too.
    ///
    /// This is the measurement the capability run is taken for, checked against
    /// the one case where the answer is known in advance. The regime is the
    /// reading a candidate pair comes off, so a table that read a cruise as a
    /// speed still rising would send a person to the wrong rule.
    #[test]
    fn capability_reads_a_trapezoid_back_as_its_own_pair() {
        // A generator that reaches its cap in the first period, so every
        // chasing period in the run travels the cap and the bands cannot
        // disagree for any reason but the walk's arithmetic.
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            PROFILE_ACCELERATION_MAX,
            PERIOD_NS,
        )
        .expect("a plant");
        let samples = stepped_chases(model, 12, 0.55, 30);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        for figure in [
            antennas.travel_p10,
            antennas.travel_p50,
            antennas.travel_p90,
            antennas.travel_max,
        ] {
            assert!(
                (figure - model.v_max).abs() < COUNT_RAD,
                "{figure} against v_max {}",
                model.v_max
            );
        }
        let readable: Vec<&super::ErrorBin> =
            antennas.bins.iter().filter(|bin| bin.readable()).collect();
        assert!(
            readable.len() >= 2,
            "the run has to fill two bands for a regime to exist: {:?}",
            antennas
                .bins
                .iter()
                .map(|bin| (bin.low, bin.samples))
                .collect::<Vec<_>>()
        );
        for bin in &readable {
            assert!(
                (bin.travel_p50 - model.v_max).abs() < COUNT_RAD,
                "band {:.2}-{:.2} reads {} against v_max {}",
                bin.low,
                bin.high,
                bin.travel_p50,
                model.v_max
            );
        }
        let Regime::MotorBound(plateau) = antennas.regime else {
            panic!("the regime is motor-bound: {:?}", antennas.regime)
        };
        assert!(
            (plateau.travel - model.v_max).abs() < COUNT_RAD,
            "plateau {} against v_max {}",
            plateau.travel,
            model.v_max
        );
        // The register figure is the pair the generator was built from, which
        // is what a candidate profile is read off.
        assert!(
            (antennas.velocity_units(plateau.travel)
                - f64::from(SHIPPED_PROFILES.antennas.velocity))
            .abs()
                < 1.0,
            "{} units",
            antennas.velocity_units(plateau.travel)
        );
        // Nothing else moved, so nothing else is measured.
        let legs = &measured[slot(JointGroup::Legs)];
        assert_eq!(legs.chasing, 0);
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability legs: no sample found this class chasing")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: motor-bound")
                    && line.contains("slowest median in it 50 units")),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "capability never judges");
    }

    /// The goal-step listing is the direct reading of both measured numbers the
    /// model rests on: the periods a joint stands still after a write, and the
    /// acceleration of its first whole period of travel.
    ///
    /// Read off a trapezoid at the shipped pair, where the ramp takes several
    /// periods and the second post-write period's gain is the generator's own
    /// acceleration. A listing whose zero periods were miscounted, or a ramp
    /// taken from the wrong pair of periods, would put the candidate
    /// acceleration out by a factor.
    #[test]
    fn a_goal_step_listing_reads_the_dead_time_and_the_ramp() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        let samples = stepped_chases(model, 12, 1.2, 80);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(antennas.steps.len(), 12, "one step per move");
        for step in &antennas.steps {
            // The write's own dead time, visible: the periods before the joint
            // answers travel nothing, and the one after them is the ramp.
            let dead = RESPONSE_DEAD_SAMPLES - 1;
            assert!(
                step.travel[..dead].iter().all(|moved| *moved == 0.0),
                "{:?} should open with {dead} still period(s)",
                step.travel
            );
            assert!(step.travel[dead] > 0.0, "{:?}", step.travel);
            let ramp = step.ramp.expect("a ramp");
            assert!(
                (ramp - model.a_max).abs() < COUNT_RAD,
                "step ramp {ramp} against a_max {}",
                model.a_max
            );
        }
        let p50 = antennas.step_ramp_p50.expect("a ramp median");
        assert!(
            (p50 - model.a_max).abs() < COUNT_RAD,
            "ramp p50 {p50} against a_max {}",
            model.a_max
        );
        assert!(
            (antennas.acceleration_units(p50) - f64::from(SHIPPED_PROFILES.antennas.acceleration))
                .abs()
                < 1.0,
            "{} units",
            antennas.acceleration_units(p50)
        );
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: 12 goal step(s), ramp p50")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: goal step at")
                    && line.contains("left antenna")),
            "the listing names the joint and the instant: {:?}",
            report.measured
        );
    }

    /// A class the run wrote too few goal steps to has its listing printed and
    /// no acceleration figure at all.
    ///
    /// The median is the class's candidate acceleration, and a median over one
    /// or two steps is the shape of one motion rather than the motor's. The
    /// listing still goes in, because a person reading two steps is reading
    /// what there is.
    #[test]
    fn a_class_written_too_few_goal_steps_has_no_acceleration_figure() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        let samples = far_chase(200, 8.0, model);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(antennas.steps.len(), 1, "one setting-off in the run");
        assert!(antennas.steps.len() < CAPABILITY_STEP_MIN_SAMPLES);
        assert!(antennas.step_ramp_p50.is_none());
        assert!(antennas.step_ramp_p90.is_none());
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: 1 goal step(s), under 10")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: goal step at")),
            "the listing is printed anyway: {:?}",
            report.measured
        );
    }

    /// A joint whose speed is its error times a gain reads gain-bound, with its
    /// bands rising all the way up.
    ///
    /// The other half of the regime reading, and the one that decides whether a
    /// class's candidate pair comes off a plateau or off a fixed band: a
    /// proportional loop never stops accelerating with its error, so no two
    /// bands read one speed and there is no plateau to take a velocity from.
    #[test]
    fn a_proportional_loop_reads_gain_bound_with_its_bands_rising() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let (gain, reach, hold, moves) = (0.03, 0.45, 90, 10);
        let mut samples = Vec::new();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        let mut goals: Vec<f64> = Vec::new();
        for n in 0..2 + moves * hold {
            samples.push(sample(n as i64, &present, &commanded));
            if n >= 2 && (n - 2) % hold == 0 {
                commanded[index] += reach;
            }
            goals.push(commanded[index]);
            if n >= RESPONSE_DEAD_SAMPLES {
                let target = goals[n - RESPONSE_DEAD_SAMPLES];
                present[index] += gain * (target - present[index]);
            }
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(antennas.regime, Regime::GainBound);
        let readable: Vec<&super::ErrorBin> =
            antennas.bins.iter().filter(|bin| bin.readable()).collect();
        assert!(readable.len() >= 3, "{}", readable.len());
        for pair in readable.windows(2) {
            assert!(
                pair[1].travel_p50 > pair[0].travel_p50,
                "band {:.2} reads {} and band {:.2} reads {}",
                pair[0].low,
                pair[0].travel_p50,
                pair[1].low,
                pair[1].travel_p50
            );
        }
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: gain-bound")),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "capability never judges");
    }

    /// A chase that never leaves the first error band reads content-bound:
    /// there is one speed on record and nothing to compare it against.
    #[test]
    fn a_chase_that_never_leaves_the_first_band_reads_content_bound() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        for n in 0..200 {
            let mut present = [0.0; ROW_COUNT];
            present[index] = f64::from(n) * 0.01;
            let mut commanded = present;
            // A gap and a half ahead the whole way: enough to be a chase, never
            // enough to leave the band it starts in.
            commanded[index] += 1.5 * CHASE_GAP_RAD;
            samples.push(sample(i64::from(n), &present, &commanded));
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(antennas.chasing > 150, "{}", antennas.chasing);
        assert_eq!(antennas.bins.len(), 1, "one band, the first");
        assert_eq!(antennas.regime, Regime::ContentBound);
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: content-bound")
                    && line.contains("fewer than two readable error bands")),
            "{:?}",
            report.measured
        );
    }

    /// A joint standing on the newest goal is not chasing an older one that is
    /// still far away.
    ///
    /// The chasing test compares against every goal the servo could have been
    /// answering over the dead time, and not the oldest of them alone. The
    /// difference is a joint the driver has just told to stop where it is: it is
    /// not trying, and counting its travel would put the speed of an arrival
    /// into the figures a profile pair is written from.
    #[test]
    fn a_joint_that_arrived_at_the_newest_goal_is_not_chasing_an_older_one() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        // Two runs of the same motion: in one the driver writes the joint's own
        // position for a single period, in the other it holds the far goal
        // throughout.
        let build = |interrupt: bool| {
            let mut samples = Vec::new();
            let mut present = [0.0; ROW_COUNT];
            for n in 0..60 {
                let mut commanded = [0.0; ROW_COUNT];
                commanded[index] = if interrupt && n == 30 {
                    present[index]
                } else {
                    4.0
                };
                samples.push(sample(n, &present, &commanded));
                present[index] += 0.05;
            }
            capability(&samples, grid())[slot(JointGroup::Antennas)].chasing
        };
        let held = build(false);
        let interrupted = build(true);
        assert!(held > 40, "the control run chases throughout: {held}");
        assert_eq!(
            held - interrupted,
            RESPONSE_DEAD_SAMPLES,
            "one goal written at the joint's own position takes the samples it could have been \
             answered at out of the chase: {held} against {interrupted}"
        );
    }

    /// The recorded capability figures are the ones a re-read is compared
    /// against, so they are written out here rather than read back off the
    /// constants.
    ///
    /// Transcribed by hand from the instrument's re-read of the kept tour; an
    /// expectation computed from the same constant cannot see a mistyped digit
    /// or a class wired to another class's pair. Which figure is which is the
    /// pair's own field names, so this case is about the digits alone.
    #[test]
    fn each_class_carries_its_own_recorded_capability_pair() {
        let pair = |acceleration, velocity| ProfilePair {
            acceleration,
            velocity,
        };
        assert_eq!(RECORDED_CAPABILITY_LEGS, pair(287, 326));
        assert_eq!(RECORDED_CAPABILITY_ANTENNAS, pair(522, 640));
        assert_eq!(RECORDED_CAPABILITY_BODY_YAW, pair(20, 48));
        assert_eq!(recorded_capability(JointGroup::Legs), pair(287, 326));
        assert_eq!(recorded_capability(JointGroup::Antennas), pair(522, 640));
        assert_eq!(recorded_capability(JointGroup::BodyYaw), pair(20, 48));
    }

    /// Content the joint keeps up with is not a capability measurement: no
    /// setpoint ever stands far enough away for the joint to be trying, so the
    /// class reports nothing and says why.
    #[test]
    fn content_the_joint_keeps_up_with_measures_nothing() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        for n in 0..500 {
            let mut rows = [0.0; ROW_COUNT];
            rows[index] = f64::from(n) * 0.01;
            samples.push(sample(i64::from(n), &rows, &rows));
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(
            antennas.chasing, 0,
            "a setpoint the joint is standing on is no chase"
        );
        assert!(antennas.bins.is_empty());
        assert_eq!(antennas.increases, 0);
        assert_eq!(antennas.regime, Regime::ContentBound);
    }

    /// A joint dragged along at per-period travels somebody chose, chasing a
    /// setpoint far enough away to be trying the whole way.
    ///
    /// The counterpart of `far_chase`, whose trapezoid runs at one speed: here
    /// the travels spread, which is what makes a percentile a percentile and
    /// not four readings of the same cruise.
    fn throttled_chase(
        joint: JointRef,
        periods: usize,
        steps: &[f64],
    ) -> Vec<Logged<PoseSampleWire>> {
        let index = row(joint).expect("a bus row");
        let goal = 1_000.0;
        let mut samples = Vec::new();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        commanded[index] = goal;
        for n in 0..periods {
            samples.push(sample(n as i64, &present, &commanded));
            present[index] += steps[n % steps.len()];
        }
        samples
    }

    /// The percentiles are the ranks they are named for, over a series whose
    /// values are all different.
    ///
    /// Directly, because every stream a capability pass is checked over runs at
    /// one speed for most of its length: a swapped p10 and p90, or a rank off by
    /// one, reads identically there and decides a profile pair here.
    #[test]
    fn a_percentile_is_the_rank_it_names() {
        let ranked: Vec<f64> = (1..=10).map(f64::from).collect();
        assert!((percentile(&ranked, 0.10) - 1.0).abs() < f64::EPSILON);
        assert!((percentile(&ranked, 0.50) - 5.0).abs() < f64::EPSILON);
        assert!((percentile(&ranked, 0.90) - 9.0).abs() < f64::EPSILON);
        assert!((percentile(&ranked, 0.999) - 10.0).abs() < f64::EPSILON);
        // A series of one is every percentile of itself, and a series of none
        // is no figure rather than an index into nothing.
        assert!((percentile(&[3.5], 0.10) - 3.5).abs() < f64::EPSILON);
        assert!((percentile(&[], 0.50)).abs() < f64::EPSILON);
    }

    /// Travels that spread come back as an ordering.
    ///
    /// Which end of the spread a candidate pair is taken from is the whole
    /// decision, and every other stream a capability pass is checked over runs
    /// at one speed for most of its length: a swapped p10 and p90 reads
    /// identically there.
    #[test]
    fn a_spread_chase_reads_back_as_ranks() {
        let samples = throttled_chase(JointRef::AntennaLeft, 600, &[0.01, 0.02, 0.03, 0.04, 0.05]);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(
            antennas.chasing >= 500,
            "the joint chased the whole run: {}",
            antennas.chasing
        );
        assert!(
            antennas.travel_p10 < antennas.travel_p50
                && antennas.travel_p50 < antennas.travel_p90
                && antennas.travel_p90 <= antennas.travel_max,
            "p10 {} p50 {} p90 {} max {}",
            antennas.travel_p10,
            antennas.travel_p50,
            antennas.travel_p90,
            antennas.travel_max,
        );
        assert!((antennas.travel_p10 - 0.01).abs() < 1e-9);
        assert!((antennas.travel_p50 - 0.03).abs() < 1e-9);
        assert!((antennas.travel_p90 - 0.05).abs() < 1e-9);

        let mut report = Report::default();
        capabilities(&measured, &mut report);
        // The limit the figures are printed against is the class's own: the
        // antennas are a different XL330 variant from the seven head servos,
        // and a pair chosen against the wrong one is refused by the servo.
        assert!(
            report.measured.iter().any(|line| line.contains(&format!(
                "capability antennas: {} chasing sample(s)",
                antennas.chasing
            )) && line.contains(&format!(
                "against a recorded Velocity Limit of {RECORDED_VELOCITY_LIMIT_ANTENNAS} units"
            ))),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "capability never judges");
    }

    /// A joint whose goal never moved was written no goal step, so the class
    /// has travel figures and no acceleration figure -- and the head's figures
    /// are printed against the head's own recorded limit.
    #[test]
    fn a_chase_under_a_goal_that_never_moved_has_no_acceleration_figure() {
        let samples = throttled_chase(JointRef::Leg0, 40, &[0.05]);
        let measured = capability(&samples, grid());
        let legs = &measured[slot(JointGroup::Legs)];
        assert!(legs.chasing > 30, "{}", legs.chasing);
        assert!(legs.steps.is_empty(), "the goal stood still all run");
        assert!(legs.step_ramp_p50.is_none());
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability legs: 0 goal step(s), under 10")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability legs:")
                    && line.contains(&format!(
                        "against a recorded Velocity Limit of {RECORDED_VELOCITY_LIMIT_HEAD} units"
                    ))),
            "{:?}",
            report.measured
        );
    }

    /// A band the content barely reached is printed and marked unread rather
    /// than dropped, and the regime is read off the bands that were.
    ///
    /// The absence is a reading: it says the library never held this class that
    /// far behind. A table that dropped it would leave a person unable to tell
    /// that from a class the run never pushed at all, and a regime read across
    /// it would turn a handful of samples into a plateau.
    #[test]
    fn a_band_under_the_minimum_is_printed_unread_and_read_past() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            PROFILE_ACCELERATION_MAX,
            PERIOD_NS,
        )
        .expect("a plant");
        // Twelve moves fill the bands under 0.55 rad; one longer move puts a
        // few samples in the bands above them and no more.
        let mut samples = stepped_chases(model, 12, 0.55, 30);
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut present = [0.0; ROW_COUNT];
        present[index] = 100.0;
        let mut commanded = present;
        commanded[index] += 0.95;
        let base = samples.len() as i64 + 4;
        for n in 0..6 {
            samples.push(sample(base + n, &present, &commanded));
            present[index] += model.v_max;
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        let unread: Vec<(f64, usize)> = antennas
            .bins
            .iter()
            .filter(|bin| !bin.readable() && bin.samples > 0)
            .map(|bin| (bin.low, bin.samples))
            .collect();
        assert!(!unread.is_empty(), "{:?}", unread);
        assert!(
            unread.iter().all(|(_, n)| *n < CAPABILITY_BIN_MIN_SAMPLES),
            "{unread:?}"
        );
        assert!(
            matches!(antennas.regime, Regime::MotorBound(_)),
            "the regime is the readable bands', not the sparse ones': {:?}",
            antennas.regime
        );
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report.measured.iter().any(|line| line
                .contains("capability antennas: error 0.90–1.00 rad")
                && line.contains("unread -- under 20 samples")),
            "{:?}",
            report.measured
        );
    }

    /// One readable error band, `samples` of them all at one speed.
    ///
    /// Hand-built, because what the cases below are about is what a table of
    /// bands reads as: a stream that filled the bands in a chosen order would
    /// put the walk's own arithmetic under test at the same time, and the
    /// orders these cases need are ones no generator produces on purpose.
    fn band(index: usize, samples: usize, travel: f64) -> ErrorBin {
        ErrorBin {
            low: (index + 1) as f64 * CHASE_GAP_RAD,
            high: (index + 2) as f64 * CHASE_GAP_RAD,
            samples,
            travel_p10: travel,
            travel_p50: travel,
            travel_p90: travel,
        }
    }

    /// A class whose reading is those bands', with every figure the band table
    /// does not decide left at nothing.
    fn class_over(bins: Vec<ErrorBin>) -> ClassCapability {
        let (regime, fell_away) = regime_of(&bins);
        ClassCapability {
            group: JointGroup::Antennas,
            chasing: bins.iter().map(|bin| bin.samples).sum(),
            travel_p10: 0.0,
            travel_p50: 0.0,
            travel_p90: 0.0,
            travel_max: 0.0,
            bins,
            regime,
            fell_away,
            increases: 0,
            increase_p50: 0.0,
            increase_p90: 0.0,
            steps: Vec::new(),
            unreadable: 0,
            step_ramp_p50: None,
            step_ramp_p90: None,
            period_ns: PERIOD_NS,
        }
    }

    /// A top band slower than the one below it by more than the ratio is read
    /// past, and the regime comes off the bands under it.
    ///
    /// A motor at its ceiling holds a speed as its error grows, so a band that
    /// *lost* speed is never a ceiling reading: it is the periods after a large
    /// goal step, or a stall. Reading it as the plateau would offer a candidate
    /// Profile Velocity at a ramp speed -- under the cruise the class actually
    /// held, which is the wrong side for nothing and the wrong figure for a
    /// person.
    #[test]
    fn a_top_band_that_fell_away_is_read_past_and_the_plateau_is_under_it() {
        let cruise = 0.05;
        let class = class_over(vec![
            band(0, 40, cruise),
            band(1, 40, cruise),
            band(2, 40, cruise),
            band(3, 40, 0.5 * cruise),
        ]);
        let Regime::MotorBound(plateau) = class.regime else {
            panic!("the bands under the fall are one speed: {:?}", class.regime)
        };
        assert_eq!(plateau.bins, 3, "the fall is read past, not walked into");
        assert!(
            (plateau.high - 4.0 * CHASE_GAP_RAD).abs() < 1e-12,
            "the plateau tops out at the third band: {plateau:?}"
        );
        assert!(
            (plateau.travel - cruise).abs() < 1e-12,
            "the plateau's figure is the cruise: {plateau:?}"
        );
        assert_eq!(class.fell_away.len(), 1, "{:?}", class.fell_away);
        let mut report = Report::default();
        travel_against_error(&class, &mut report);
        assert!(
            report.measured.iter().any(|line| {
                line.contains("capability antennas: error 0.40–0.50 rad, n 40")
                    && line.contains(&format!(
                        "travel p50 {:.0} units -- fell away",
                        class.velocity_units(0.5 * cruise)
                    ))
                    && line.contains("2.00 times slower")
            }),
            "the fallen band is named with its own figures: {:?}",
            report.measured
        );
        assert!(
            report.measured.iter().any(|line| {
                line.contains("capability antennas: motor-bound")
                    && line.contains("over 3 band(s)")
                    && line.contains(&format!(
                        "slowest median in it {:.0} units",
                        class.velocity_units(cruise)
                    ))
                    && line.contains("read under 1 band(s) that fell away")
            }),
            "{:?}",
            report.measured
        );
    }

    /// A band whose median came out zero is one speed with nothing, so a joint
    /// that stalled while more than a gap behind its goal is not folded into a
    /// plateau at zero units.
    ///
    /// The rule lives in `one_speed`'s arithmetic -- a ratio that is not a
    /// number is not less than the ratio -- so it is asserted directly as well
    /// as through a table. The natural-looking rewrite to a difference over a
    /// maximum makes a zero-against-zero pair "one speed" and offers a
    /// candidate Profile Velocity of zero, which is the figure
    /// `PlantModel::from_registers` refuses.
    #[test]
    fn a_band_that_read_zero_is_one_speed_with_nothing() {
        let cruise = 0.05;
        assert!(!one_speed(0.0, 0.0), "a stall is not a speed");
        assert!(!one_speed(0.0, cruise));
        assert!(!one_speed(cruise, 0.0));
        assert!(one_speed(cruise, cruise));
        assert!(one_speed(
            cruise,
            cruise * (CAPABILITY_PLATEAU_RATIO - 0.05)
        ));
        assert!(!one_speed(
            cruise,
            cruise * (CAPABILITY_PLATEAU_RATIO + 0.05)
        ));

        // A stalled band under a cruise: the plateau is the cruise's two bands
        // and the stall is below its foot.
        let stalled = class_over(vec![
            band(0, 40, 0.0),
            band(1, 40, cruise),
            band(2, 40, cruise),
        ]);
        let Regime::MotorBound(plateau) = stalled.regime else {
            panic!("{:?}", stalled.regime)
        };
        assert_eq!(plateau.bins, 2, "{plateau:?}");
        assert!((plateau.travel - cruise).abs() < 1e-12, "{plateau:?}");
        assert!(stalled.fell_away.is_empty(), "{:?}", stalled.fell_away);

        // A stalled *top* band is a fall at its extreme, so it is read past and
        // the plateau is the cruise under it. No zero reaches a plateau figure.
        let topped = class_over(vec![
            band(0, 40, cruise),
            band(1, 40, cruise),
            band(2, 40, 0.0),
        ]);
        let Regime::MotorBound(plateau) = topped.regime else {
            panic!("{:?}", topped.regime)
        };
        assert_eq!(plateau.bins, 2, "{plateau:?}");
        assert!((plateau.travel - cruise).abs() < 1e-12, "{plateau:?}");
        assert_eq!(topped.fell_away.len(), 1, "{:?}", topped.fell_away);
        let mut report = Report::default();
        travel_against_error(&topped, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("fell away") && line.contains("read zero")),
            "a stalled band has no ratio to print: {:?}",
            report.measured
        );
    }

    /// Two stalled bands over a cruise are both read past, and the plateau is
    /// the cruise.
    ///
    /// The case the walk's condition has to be a complement for: `one_speed`
    /// refuses a zero against a zero and `>` does not order it either, so a
    /// rule written as "not one speed and slower" catches the pair and a rule
    /// written as "slower" alone would stop the walk on the upper zero and read
    /// the plateau at zero units.
    #[test]
    fn two_stalled_bands_over_a_cruise_are_both_read_past() {
        let cruise = 0.05;
        let class = class_over(vec![
            band(0, 40, cruise),
            band(1, 40, cruise),
            band(2, 40, 0.0),
            band(3, 40, 0.0),
        ]);
        let Regime::MotorBound(plateau) = class.regime else {
            panic!("{:?}", class.regime)
        };
        assert_eq!(plateau.bins, 2, "{plateau:?}");
        assert!((plateau.travel - cruise).abs() < 1e-12, "{plateau:?}");
        assert_eq!(class.fell_away.len(), 2, "{:?}", class.fell_away);
    }

    /// Two successive falls leave one band, and a class read content-bound with
    /// bands read past is no measurement of that class.
    ///
    /// The content-bound candidate rule extrapolates a pair above everything
    /// observed, on the premise that the library never held the class far
    /// behind. A class whose far bands were read past was held exactly there,
    /// so the premise is false and the report says the run offers no candidate
    /// rather than repeating the never-held sentence.
    #[test]
    fn a_class_read_content_bound_after_a_fall_offers_no_candidate() {
        let cruise = 0.05;
        let class = class_over(vec![
            band(0, 40, cruise),
            band(1, 40, 0.6 * cruise),
            band(2, 40, 0.3 * cruise),
        ]);
        assert_eq!(class.regime, Regime::ContentBound, "{:?}", class.regime);
        assert_eq!(class.fell_away.len(), 2, "{:?}", class.fell_away);
        let mut report = Report::default();
        travel_against_error(&class, &mut report);
        let fallen = report
            .measured
            .iter()
            .filter(|line| line.contains("fell away"))
            .count();
        assert_eq!(fallen, 2, "{:?}", report.measured);
        assert!(
            report.measured.iter().any(|line| {
                line.contains("capability antennas: content-bound")
                    && line.contains("the reading needs two speeds")
                    && line.contains("2 band(s) above were read past")
                    && line.contains("not a measurement of this class")
            }),
            "{:?}",
            report.measured
        );
        assert!(
            !report
                .measured
                .iter()
                .any(|line| line.contains("fewer than two readable error bands")),
            "the never-held sentence is a different reading: {:?}",
            report.measured
        );
    }

    /// One readable band and nothing read past still reads as the content never
    /// having pushed the class.
    ///
    /// The other half of the case above: the two content-bound lines say
    /// different things, and which one prints turns on whether a band was set
    /// aside.
    #[test]
    fn content_bound_with_nothing_read_past_still_says_the_content_never_pushed() {
        let class = class_over(vec![band(0, 40, 0.05), band(1, 4, 0.05)]);
        assert_eq!(class.regime, Regime::ContentBound, "{:?}", class.regime);
        assert!(class.fell_away.is_empty(), "{:?}", class.fell_away);
        let mut report = Report::default();
        travel_against_error(&class, &mut report);
        assert!(
            report.measured.iter().any(|line| {
                line.contains("capability antennas: content-bound")
                    && line.contains("fewer than two readable error bands")
            }),
            "{:?}",
            report.measured
        );
    }

    /// A fall over a rising remainder is read past and the remainder reads
    /// gain-bound.
    ///
    /// The fell-away walk runs before the regime is named, so a run whose top
    /// band is ramp periods does not hide a loop that was still speeding up
    /// with its error underneath it.
    #[test]
    fn a_fall_over_a_rising_remainder_reads_gain_bound() {
        let step = 0.01;
        let class = class_over(vec![
            band(0, 40, step),
            band(1, 40, 2.0 * step),
            band(2, 40, 4.0 * step),
            band(3, 40, 8.0 * step),
            band(4, 40, 6.0 * step),
        ]);
        assert_eq!(class.regime, Regime::GainBound, "{:?}", class.regime);
        assert_eq!(class.fell_away.len(), 1, "{:?}", class.fell_away);
        let mut report = Report::default();
        travel_against_error(&class, &mut report);
        assert!(
            report.measured.iter().any(|line| {
                line.contains("capability antennas: gain-bound")
                    && line.contains("read under 1 band(s) that fell away")
            }),
            "{:?}",
            report.measured
        );
    }

    /// A top band slower than the one under it but inside the ratio is one
    /// speed with it and stays in the plateau.
    ///
    /// The fall is the ratio and not the sign: two bands within
    /// `CAPABILITY_PLATEAU_RATIO` are two errors the class answered at the same
    /// speed, whichever came out the faster, and a rule keyed on the sign alone
    /// would cut the plateau's own top band off it.
    #[test]
    fn a_top_band_slower_inside_the_ratio_is_one_speed_and_stays() {
        let cruise = 0.05;
        let class = class_over(vec![
            band(0, 40, cruise),
            band(1, 40, cruise),
            band(2, 40, 0.9 * cruise),
        ]);
        let Regime::MotorBound(plateau) = class.regime else {
            panic!("{:?}", class.regime)
        };
        assert_eq!(plateau.bins, 3, "{plateau:?}");
        assert!(
            (plateau.travel - 0.9 * cruise).abs() < 1e-12,
            "the plateau's figure is its slowest median: {plateau:?}"
        );
        assert!(class.fell_away.is_empty(), "{:?}", class.fell_away);
    }

    /// A goal that moved less than a chase gap is no goal step: it does not put
    /// the joint into a chase, so what follows it is an arrival and not a ramp.
    ///
    /// The median of the steps' ramps is the class's candidate Profile
    /// Acceleration, so a listing that admitted arrivals would bias a figure
    /// that ends up in a servo register.
    #[test]
    fn a_goal_moved_less_than_a_chase_gap_is_no_step() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        let samples = stepped_chases(model, 12, 0.5 * CHASE_GAP_RAD, 30);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(
            antennas.steps.is_empty(),
            "twelve sub-gap goal moves are no steps: {:?}",
            antennas
                .steps
                .iter()
                .map(|step| step.step)
                .collect::<Vec<_>>()
        );
        assert!(antennas.step_ramp_p50.is_none());
        // The half-gap that is no step is no chase either, which is the reason
        // the rejection exists: the class measured nothing, and the report says
        // that rather than printing a listing of arrivals.
        assert_eq!(antennas.chasing, 0);
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report.measured.iter().any(|line| line.contains(
                "capability antennas: no sample found this class chasing a setpoint more than \
                 0.1 rad away"
            )),
            "{:?}",
            report.measured
        );
    }

    /// A goal step written to a joint that was already travelling is not
    /// listed: it ramps from the speed it had rather than from rest, so its
    /// second period's gain is not the motor's acceleration.
    ///
    /// Both halves of the rejection matter and this is the other one. The
    /// stream holds one write from rest and one made two periods into the
    /// answer, and only the first is a reading.
    #[test]
    fn a_goal_step_written_mid_travel_is_not_listed() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let (from_rest, mid_travel) = (2_i64, 8_i64);
        let mut samples = Vec::new();
        let mut state = Predicted::default();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        let mut goals: Vec<f64> = Vec::new();
        for n in 0..60_i64 {
            samples.push(sample(n, &present, &commanded));
            if n == from_rest || n == mid_travel {
                commanded[index] += 3.0;
            }
            goals.push(commanded[index]);
            let at = usize::try_from(n).expect("a sample index");
            if at >= RESPONSE_DEAD_SAMPLES {
                model.step(&mut state, goals[at - RESPONSE_DEAD_SAMPLES]);
                present[index] = state.position;
            }
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(
            antennas.steps.len(),
            1,
            "only the write the joint was standing still for: {:?}",
            antennas
                .steps
                .iter()
                .map(|step| step.at_ns - ORIGIN_NS)
                .collect::<Vec<_>>()
        );
        // The instant is the sample the goal changed at, which is the period
        // after the write in this stream's own shape.
        assert_eq!(
            antennas.steps[0].at_ns,
            ORIGIN_NS + (from_rest + 1) * PERIOD_NS
        );
        assert!(
            antennas.steps[0].ramp.is_some(),
            "the from-rest step still carries its ramp"
        );
    }

    /// A stretch that ended inside a step has no ramp for it, and says so
    /// rather than printing a figure it did not measure.
    ///
    /// What a run cut mid-move produces -- a fetch whose log stops, a stretch
    /// broken by a dropped sample. The `None` is also what stands between the
    /// report and a median over no ramps at all: a class written enough steps
    /// that none of them carried one has no acceleration figure, not a figure
    /// of zero.
    #[test]
    fn a_step_the_stretch_ended_inside_has_no_ramp() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        // Twelve moves, then the stream cut one period after the last write's
        // own sample: the step is read and the two periods its ramp is the
        // difference of are not both there.
        let hold = 80;
        let mut samples = stepped_chases(model, 12, 1.2, hold);
        samples.truncate(2 + 11 * hold + 3);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(antennas.steps.len(), 12, "every write is still a step");
        let last = antennas.steps.last().expect("a step");
        assert!(
            last.ramp.is_none(),
            "the stretch ended inside it: {:?}",
            last.travel
        );
        assert_eq!(last.travel.len(), 1, "one period after the write, not two");
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: goal step at")
                    && line.contains("ramp no ramp -- the stretch ended inside the step")),
            "{:?}",
            report.measured
        );
        // The eleven whole steps still carry the median, so this class's
        // acceleration is read past the cut one.
        assert!(antennas.step_ramp_p50.is_some());

        // And a class whose steps all ended their stretches has no median at
        // all: twelve two-sample stretches, each a write and one sample after
        // it, with a gap between them.
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut cut = Vec::new();
        // The joint stands where it is throughout: a step is a write made to a
        // joint that was not travelling, and what this stream is about is the
        // periods after the write that are not there.
        let present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        for burst in 0..12_i64 {
            // Four slots apart, which is past MAX_GAP_PERIODS, so no two
            // bursts are one stretch.
            let at = burst * (MAX_GAP_PERIODS as i64 + 4);
            cut.push(sample(at, &present, &commanded));
            commanded[index] += 3.0;
            cut.push(sample(at + 1, &present, &commanded));
        }
        let stepless = capability(&cut, grid());
        let antennas = &stepless[slot(JointGroup::Antennas)];
        assert_eq!(antennas.steps.len(), 12);
        assert!(
            antennas.steps.iter().all(|step| step.ramp.is_none()),
            "no burst holds two post-write periods"
        );
        assert!(
            antennas.step_ramp_p50.is_none() && antennas.step_ramp_p90.is_none(),
            "twelve steps and no ramp among them is no figure, not a zero: {:?} {:?}",
            antennas.step_ramp_p50,
            antennas.step_ramp_p90
        );
    }

    /// The increase figures are the acceleration a class that never reached its
    /// motor is read at, and over a joint accelerating by a known amount every
    /// period they are that amount.
    ///
    /// `RECORDED_CAPABILITY_BODY_YAW`'s acceleration is this median -- the yaw
    /// is written too few goal steps for a ramp median -- so an off-by-one in
    /// the period pairing, or the loss of the encoder-count filter, moves a
    /// figure Step B writes into a servo. A linear ramp catches both: a
    /// mis-paired difference reads twice the gain or none of it.
    #[test]
    fn the_increase_median_is_the_acceleration_of_a_known_ramp() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        // A joint whose per-period travel grows by `gain` every period, chasing
        // a goal it never approaches.
        let ramped = |gain: f64, periods: i64| {
            let mut samples = Vec::new();
            let mut present = [0.0; ROW_COUNT];
            let mut commanded = [0.0; ROW_COUNT];
            commanded[index] = 1_000.0;
            for n in 0..periods {
                samples.push(sample(n, &present, &commanded));
                present[index] += gain * (n + 1) as f64;
            }
            samples
        };
        let gain = 0.01;
        assert!(gain > COUNT_RAD, "the gain has to clear the filter");
        let measured = capability(&ramped(gain, 40), grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(
            antennas.increases,
            40 - RESPONSE_DEAD_SAMPLES,
            "every chasing period after the first gained on the one before it"
        );
        for figure in [antennas.increase_p50, antennas.increase_p90] {
            assert!(
                (figure - gain).abs() < 1e-12,
                "{figure} against a per-period gain of {gain}"
            );
        }
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report.measured.iter().any(|line| line.contains(&format!(
                "capability antennas: {} chasing period(s) travelled at least a count further",
                antennas.increases
            )) && line
                .contains(&format!("({:.0} units)", antennas.acceleration_units(gain)))),
            "{:?}",
            report.measured
        );

        // A ramp under an encoder count a period is filtered out: the figure is
        // the acceleration of a joint that is accelerating, and a count is the
        // smallest change the encoder can tell from stillness.
        let under = capability(&ramped(0.5 * COUNT_RAD, 40), grid());
        let antennas = &under[slot(JointGroup::Antennas)];
        assert!(antennas.chasing > 30, "the joint chased throughout");
        assert_eq!(
            antennas.increases, 0,
            "a gain under a count is encoder noise, not acceleration"
        );
    }

    /// A non-finite reading in a log is corruption, not a fast joint: it is
    /// counted, left out of every figure, and named in the report.
    ///
    /// It passes the chase test -- anything stands further than a gap from
    /// infinity -- and the error band it would be filed under is at the top of
    /// the address space, so binning it is an allocation the analyzer dies on.
    /// A whole tour's report, health and residuals included, is lost with it.
    #[test]
    fn a_non_finite_reading_is_counted_and_measures_nothing() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        let mut present = [0.0; ROW_COUNT];
        let mut commanded = [0.0; ROW_COUNT];
        commanded[index] = 1_000.0;
        for n in 0..40 {
            present[index] = if n == 20 {
                f64::INFINITY
            } else {
                f64::from(n) * 0.05
            };
            samples.push(sample(i64::from(n), &present, &commanded));
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        // One infinite reading is in two periods' travel -- the one that
        // reached it and the one that left it -- and in the second of those
        // periods' error as well. Neither sample measures anything; the
        // thirty-seven around them do.
        assert_eq!(antennas.unreadable, 2, "{}", antennas.unreadable);
        assert_eq!(antennas.chasing, 40 - RESPONSE_DEAD_SAMPLES - 2);
        for figure in [
            antennas.travel_p10,
            antennas.travel_p50,
            antennas.travel_p90,
            antennas.travel_max,
            antennas.increase_p50,
            antennas.increase_p90,
        ] {
            assert!(figure.is_finite(), "{figure}");
        }
        assert!(
            antennas.bins.iter().all(|bin| bin.travel_p50.is_finite()),
            "no band holds it"
        );
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report.measured.iter().any(|line| line
                .contains("capability antennas: 2 chasing sample(s) carried a non-finite reading")),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "capability never judges");
    }

    /// A gap in the sample stream is not a period: the travel across it is two
    /// or more periods' motion, and reading it as one would report a motor
    /// faster than the machine holds.
    #[test]
    fn a_gap_in_the_stream_is_not_a_per_period_travel() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.velocity,
            SHIPPED_PROFILES.antennas.acceleration,
            PERIOD_NS,
        )
        .expect("a plant");
        let whole = far_chase(60, 8.0, model);
        let mut gapped = far_chase(60, 8.0, model);
        gapped.remove(30);
        let across = capability(&gapped, grid())[slot(JointGroup::Antennas)].travel_max;
        let straight = capability(&whole, grid())[slot(JointGroup::Antennas)].travel_max;
        // Both figures are the generator's own cruise: an equality between two
        // passes that measured nothing would be the same assertion and would
        // hold with the walk taken out altogether.
        assert!(
            (straight - model.v_max).abs() < COUNT_RAD,
            "the ungapped run cruises at the generator's speed: {straight} against {}",
            model.v_max
        );
        assert!(
            (across - straight).abs() < f64::EPSILON,
            "the two-period step across the hole was counted: {across} against {straight}"
        );
    }
}
