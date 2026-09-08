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
use reachy_motion::joints::{JointGroup, ROWS, group_of, row, rows_of};
use reachy_motion::plant::{
    GroupPlants, GroupProfiles, MAX_GAP_PERIODS, PROFILE_ACCELERATION_UNIT_RAD_PER_S2,
    PROFILE_VELOCITY_UNIT_RAD_PER_S, Predicted, RESPONSE_DEAD_SAMPLES,
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
        Ok((
            figure(&format!("{class}_profile_acceleration"))?,
            figure(&format!("{class}_profile_velocity"))?,
        ))
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
    /// That the directory is not there — which is a run recorded before the
    /// copy existed, read with its own build — or the reason one of the three
    /// files could not be read or does not state a name.
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
    plant: &GroupPlants,
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
                        plant.for_row(index).step(state, target[index]);
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
    for (_, residual) in stream {
        for joint in ROWS {
            let (Some(index), Some(group)) = (row(joint), group_of(joint)) else {
                continue;
            };
            seen[slot(group)].push(residual[index]);
        }
    }
    for group in JointGroup::ALL {
        let model = plant.of(group);
        let ranked = &mut seen[slot(group)];
        ranked.sort_by(f64::total_cmp);
        let worst = ranked.last().copied().unwrap_or(0.0);
        let p999 = percentile(ranked, 0.999);
        let (low, high) = recorded_p999_range(group);
        report.note(format!(
            "{}: worst residual {worst:.4} rad, p99.9 {p999:.4} rad, recorded p99.9 under the \
             shipped pair {low:.4}–{high:.4} rad over three tours, against a tracking screen \
             at {threshold:.4} rad — commissioned at {:.6} rad/period and {:.6} rad/period²",
            group.name(),
            model.v_max,
            model.a_max,
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
/// The gap the model fit was made over: past it the joint is moving as fast as
/// it is going to, so the travel it makes in that period is what its motor
/// achieves rather than what the content asked for. Under it the joint is
/// arriving, and its travel says only how close it already was.
pub const CHASE_GAP_RAD: f64 = 0.1;

/// How many periods a ramp is looked at over, at most.
///
/// A window rather than an averaging length: what is read out of it is the
/// largest single period's gain inside it, and the window stops at the first
/// period the joint stopped gaining in. A mean over a fixed four periods would
/// read a motor that reached its cap in one period as a quarter of its own
/// acceleration -- which is exactly the motor a capability run at a profile
/// above the machine's own is taken to measure.
pub const RAMP_PERIODS: usize = 4;

/// How many chasing samples a class needs before its figures describe its
/// motor rather than the content.
///
/// A tour is tens of thousands of samples, so a motor that is the binding
/// constraint anywhere in the library clears this easily; a class that does not
/// is one the library never asked more of than it could give, and its figures
/// are the content's speed rather than the motor's. Which of the two a class is
/// in is what a candidate profile pair is chosen off, so the report says it.
pub const CAPABILITY_MIN_SAMPLES: usize = 500;

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
    /// How many ramps -- a joint setting off from rest and chasing -- the
    /// increase figures came off.
    pub ramps: usize,
    /// How many of those ramps stopped gaining inside the window, so that the
    /// figure they contributed is what the joint reached rather than what it
    /// was still capable of. A class whose ramps mostly saturated is one whose
    /// acceleration figure is a floor.
    pub ramps_capped: usize,
    /// The median ramp's largest single-period increase in travel, radians per
    /// period squared.
    pub ramp_p50: f64,
    /// The increase a tenth of the ramps beat, radians per period squared.
    pub ramp_p90: f64,
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
    let mut travels: [Vec<f64>; JointGroup::ALL.len()] = Default::default();
    let mut ramps: [Vec<f64>; JointGroup::ALL.len()] = Default::default();
    let mut capped: [usize; JointGroup::ALL.len()] = Default::default();
    for stretch in stretches(&ordered, grid) {
        walk_stretch(&stretch, &mut travels, &mut ramps, &mut capped);
    }
    JointGroup::ALL.map(|group| {
        let travel = &mut travels[slot(group)];
        let ramp = &mut ramps[slot(group)];
        travel.sort_by(f64::total_cmp);
        ramp.sort_by(f64::total_cmp);
        ClassCapability {
            group,
            chasing: travel.len(),
            travel_p10: percentile(travel, 0.10),
            travel_p50: percentile(travel, 0.50),
            travel_p90: percentile(travel, 0.90),
            travel_max: travel.last().copied().unwrap_or(0.0),
            ramps: ramp.len(),
            ramps_capped: capped[slot(group)],
            ramp_p50: percentile(ramp, 0.50),
            ramp_p90: percentile(ramp, 0.90),
            period_ns: grid.period_ns,
        }
    })
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

/// The body yaw's p99.9 residual under the shipped `20 / 50` pair, one figure
/// per tour, in tour order.
///
/// Three figures and not one, because what a candidate pair's p99.9 is read for
/// is *growth*, and the run-to-run spread between identically configured tours
/// is the noise floor of that reading: a candidate landing inside this range has
/// not grown, one above its top has, by at least the amount above. A single
/// tour's figure would have every reader mistaking the spread for a change.
///
/// Read over three tours of the whole library at the shipped profile, the
/// shipped gains and an identical clip library; provenance in
/// `docs/servo-tuning.md`.
///
/// The other family of recorded residual figures off those same tours is the
/// worst per joint group, `RECORDED_WORST_HEAD_RESIDUAL_RAD` and
/// `RECORDED_WORST_ANTENNA_RESIDUAL_RAD` in `reachy_motion::tick`, which is
/// what the tracking screen is sized on. A confirmation tour that re-bakes
/// those worsts re-bakes these three arrays from the same tour, or the report
/// prints a fresh worst beside a noise floor measured under a superseded
/// profile.
pub const RECORDED_P999_BODY_YAW_RESIDUAL_RAD: [f64; 3] = [0.2570, 0.3050, 0.2475];

/// The legs' p99.9 residual over the same three tours, in the same order.
pub const RECORDED_P999_LEGS_RESIDUAL_RAD: [f64; 3] = [0.2083, 0.1976, 0.1925];

/// The antennas' p99.9 residual over the same three tours, in the same order.
pub const RECORDED_P999_ANTENNAS_RESIDUAL_RAD: [f64; 3] = [0.3096, 0.2745, 0.2962];

/// The three recorded p99.9 residuals of one class.
///
/// Three and not a slice: the arity is what makes the range below a range of
/// figures that exist, so a class that lost its tours cannot fold to a
/// sentinel.
fn recorded_p999(group: JointGroup) -> &'static [f64; 3] {
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
    let [first, second, third] = *recorded_p999(group);
    (first.min(second).min(third), first.max(second).max(third))
}

/// One sample's two nine-row readings, as a capability walk needs them.
type Step = ([f64; ROWS.len()], [f64; ROWS.len()]);

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
        let (cycle, _) = grid.at(sample.message.nominal_time().as_nanos());
        let consecutive = previous.is_some_and(|before| cycle == before + 1);
        let step = present_rows(&sample.message).zip(commanded_rows(&sample.message));
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

/// Every chasing sample in one stretch, folded into the per-class figures.
fn walk_stretch(
    stretch: &[Step],
    travels: &mut [Vec<f64>; JointGroup::ALL.len()],
    ramps: &mut [Vec<f64>; JointGroup::ALL.len()],
    capped: &mut [usize; JointGroup::ALL.len()],
) {
    let travel = |at: usize, index: usize| (stretch[at].0[index] - stretch[at - 1].0[index]).abs();
    // Whether the joint was chasing at this sample: the setpoint the driver was
    // holding a dead time earlier -- which is the one the servo had had time to
    // act on -- stood further than the gap from where the joint then was.
    // A dead time is at least one period, so every sample a chase can be read
    // at has a sample before it for the travel to be measured against.
    const _: () = assert!(RESPONSE_DEAD_SAMPLES >= 1);
    let chasing = |at: usize, index: usize| {
        at >= RESPONSE_DEAD_SAMPLES
            && (stretch[at - RESPONSE_DEAD_SAMPLES].1[index] - stretch[at - 1].0[index]).abs()
                > CHASE_GAP_RAD
    };
    for at in RESPONSE_DEAD_SAMPLES..stretch.len() {
        for joint in ROWS {
            let (Some(index), Some(group)) = (row(joint), group_of(joint)) else {
                continue;
            };
            if !chasing(at, index) {
                continue;
            }
            travels[slot(group)].push(travel(at, index));
            // A ramp is a joint setting off: it stood still last period and is
            // chasing now, and what is read is the most it gained in any one
            // period after, which is the acceleration its own generator or its
            // own motor imposed. The largest single period and not the mean over
            // the window: a joint that reaches its cap in one period gains
            // nothing in the periods after, and averaging those in reports an
            // acceleration the window's length decided rather than the motor.
            // The window ends where the gaining does, for the same reason, and
            // where the chase does, because a joint arriving is slowing down.
            if at < 2 || travel(at - 1, index) >= COUNT_RAD {
                continue;
            }
            let mut best: Option<f64> = None;
            let mut stopped = false;
            // From the period it set off in: a joint that reaches its speed in
            // that one period gains nothing after it, and a window that started
            // afterwards would read the fastest motor on the bus as no
            // acceleration at all.
            for ahead in 0..=RAMP_PERIODS {
                if at + ahead >= stretch.len() || (ahead > 0 && !chasing(at + ahead, index)) {
                    break;
                }
                let gained = travel(at + ahead, index) - travel(at + ahead - 1, index);
                best = Some(best.map_or(gained, |most: f64| most.max(gained)));
                if gained < COUNT_RAD {
                    stopped = true;
                    break;
                }
            }
            if let Some(gained) = best {
                ramps[slot(group)].push(gained);
                if stopped {
                    capped[slot(group)] += 1;
                }
            }
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
pub fn capabilities(measured: &[ClassCapability; 3], report: &mut Report) {
    for class in measured {
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
        report.note(if class.chasing >= CAPABILITY_MIN_SAMPLES {
            format!(
                "capability {}: saturating -- past {CAPABILITY_MIN_SAMPLES} chasing samples, so \
                 these figures are what the motors did rather than what the content asked",
                class.group.name()
            )
        } else {
            format!(
                "capability {}: content-bound -- under {CAPABILITY_MIN_SAMPLES} chasing samples, \
                 so what the library asked of this class is what these figures measure",
                class.group.name()
            )
        });
        if class.ramps == 0 {
            report.note(format!(
                "capability {}: no ramp -- no joint of this class set off from rest into a chase \
                 that held for a period",
                class.group.name()
            ));
            continue;
        }
        report.note(format!(
            "capability {}: {} ramp(s) in a window of {RAMP_PERIODS} periods, {} of them capped \
             inside it, largest increase per period p50 {:.6} rad ({:.0} units), p90 {:.6} ({:.0})",
            class.group.name(),
            class.ramps,
            class.ramps_capped,
            class.ramp_p50,
            class.acceleration_units(class.ramp_p50),
            class.ramp_p90,
            class.acceleration_units(class.ramp_p90),
        ));
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
    use reachy_motion::plant::{PlantModel, SHIPPED_PROFILES};
    use reachy_motion::stillness::COUNT_RAD;

    use super::{
        CAPABILITY_MIN_SAMPLES, CONFIG_FILES, Grid, RECORDED_P999_ANTENNAS_RESIDUAL_RAD,
        RECORDED_P999_BODY_YAW_RESIDUAL_RAD, RECORDED_P999_LEGS_RESIDUAL_RAD,
        RECORDED_VELOCITY_LIMIT_ANTENNAS, RECORDED_VELOCITY_LIMIT_HEAD, RunConfig, Skips,
        TEMPERATURE_STOP_C, capabilities, capability, health_summary, lags, no_faults, percentile,
        residual_stream, residuals, slot,
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
    /// model was fitted at rather than about any delay at all.
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

    /// Every class's residual line carries that class's own recorded p99.9
    /// range, low figure first.
    ///
    /// The floor is at the point of use because the reading it supports is a
    /// comparison: a candidate pair's p99.9 against what the shipped pair did
    /// over three tours. A line carrying another class's range, or the range
    /// the wrong way round, would read as growth that is not there.
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
            RECORDED_P999_BODY_YAW_RESIDUAL_RAD,
            [0.2570, 0.3050, 0.2475]
        );
        assert_eq!(RECORDED_P999_LEGS_RESIDUAL_RAD, [0.2083, 0.1976, 0.1925]);
        assert_eq!(
            RECORDED_P999_ANTENNAS_RESIDUAL_RAD,
            [0.3096, 0.2745, 0.2962]
        );

        let ranges = [
            (JointGroup::BodyYaw, "0.2475–0.3050"),
            (JointGroup::Legs, "0.1925–0.2083"),
            (JointGroup::Antennas, "0.2745–0.3096"),
        ];
        for (group, range) in ranges {
            let wanted =
                format!("recorded p99.9 under the shipped pair {range} rad over three tours");
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
                     antennas_p: 500\nantennas_i: 0\nantennas_d: 100\n";
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
                legs: (20, 50),
                yaw: (30, 60),
                antennas: (40, 70),
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
        assert_eq!(config.profiles.legs, (20, 50), "0x14 is twenty");
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

    /// A joint running its own generator's trapezoid reads back as that
    /// generator: the median travel is the profile velocity and the ramp is the
    /// profile acceleration, both to within an encoder count.
    ///
    /// This is the measurement the capability run is taken for, checked against
    /// the one case where the answer is known in advance.
    #[test]
    fn capability_reads_a_trapezoid_back_as_its_own_pair() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.1,
            SHIPPED_PROFILES.antennas.0,
            PERIOD_NS,
        )
        .expect("a plant");
        let samples = far_chase(200, 8.0, model);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(
            antennas.chasing > 150,
            "the joint chased for most of the run: {}",
            antennas.chasing
        );
        assert!(
            (antennas.travel_p50 - model.v_max).abs() < COUNT_RAD,
            "p50 {} against v_max {}",
            antennas.travel_p50,
            model.v_max
        );
        assert!(
            (antennas.travel_p10 - model.v_max).abs() < COUNT_RAD,
            "p10 {} against v_max {}",
            antennas.travel_p10,
            model.v_max
        );
        assert!(
            (antennas.travel_max - model.v_max).abs() < COUNT_RAD,
            "max {} against v_max {}",
            antennas.travel_max,
            model.v_max
        );
        assert_eq!(antennas.ramps, 1, "one setting-off in the run");
        assert!(
            (antennas.ramp_p50 - model.a_max).abs() < COUNT_RAD,
            "ramp p50 {} against a_max {}",
            antennas.ramp_p50,
            model.a_max
        );
        // The register figures are the pair the generator was built from,
        // which is what a candidate profile is read off.
        assert!(
            (antennas.velocity_units(antennas.travel_p50) - f64::from(SHIPPED_PROFILES.antennas.1))
                .abs()
                < 1.0,
            "{} units",
            antennas.velocity_units(antennas.travel_p50)
        );
        assert!(
            (antennas.acceleration_units(antennas.ramp_p50)
                - f64::from(SHIPPED_PROFILES.antennas.0))
            .abs()
                < 1.0,
            "{} units",
            antennas.acceleration_units(antennas.ramp_p50)
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
                .any(|line| line.contains("capability antennas: content-bound")),
            "a 200-sample run is under the minimum: {:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "capability never judges");
    }

    /// Content the joint keeps up with is not a capability measurement: no
    /// setpoint ever stands far enough away for the joint to be trying, so the
    /// class reports nothing and says why.
    #[test]
    fn content_the_joint_keeps_up_with_measures_nothing() {
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        let mut samples = Vec::new();
        for n in 0..CAPABILITY_MIN_SAMPLES as i64 {
            let mut rows = [0.0; ROW_COUNT];
            rows[index] = n as f64 * 0.01;
            samples.push(sample(n, &rows, &rows));
        }
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert_eq!(
            antennas.chasing, 0,
            "a setpoint the joint is standing on is no chase"
        );
        assert_eq!(antennas.ramps, 0);
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

    /// Travels that spread come back as an ordering, and a class that chased
    /// past the minimum is reported as the motor's own figures.
    ///
    /// The two readings the decision tree is made of: which end of the spread a
    /// candidate pair is taken from, and whether the class was the binding
    /// constraint at all.
    #[test]
    fn a_spread_chase_reads_back_as_ranks_and_as_saturating() {
        let samples = throttled_chase(
            JointRef::AntennaLeft,
            CAPABILITY_MIN_SAMPLES + 100,
            &[0.01, 0.02, 0.03, 0.04, 0.05],
        );
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(
            antennas.chasing >= CAPABILITY_MIN_SAMPLES,
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
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability antennas: saturating")),
            "past the minimum the figures are the motor's: {:?}",
            report.measured
        );
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

    /// A joint already moving when the stream starts is chasing but never set
    /// off, so the class has travel figures and no acceleration figure -- and
    /// the head's figures are printed against the head's own recorded limit.
    #[test]
    fn a_chase_that_never_set_off_from_rest_reports_no_ramp() {
        let samples = throttled_chase(JointRef::Leg0, 40, &[0.05]);
        let measured = capability(&samples, grid());
        let legs = &measured[slot(JointGroup::Legs)];
        assert!(legs.chasing > 30, "{}", legs.chasing);
        assert_eq!(legs.ramps, 0, "nothing in the run stood still first");
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("capability legs: no ramp")),
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

    /// A motor that reaches its speed in one period is reported at that
    /// period's gain, not at a quarter of it.
    ///
    /// The window is a window and not an averaging length: a capability run is
    /// taken at a profile above what the motors can do, where every ramp ends
    /// inside it, and a mean over four periods would call that motor four times
    /// slower than it is. The count of capped ramps says the figure is a floor.
    #[test]
    fn a_ramp_that_ends_inside_the_window_is_not_averaged_over_it() {
        // Still, then one period at full speed and cruising there.
        let samples = throttled_chase(JointRef::AntennaLeft, 40, &[0.0, 0.2, 0.2, 0.2, 0.2, 0.2]);
        let measured = capability(&samples, grid());
        let antennas = &measured[slot(JointGroup::Antennas)];
        assert!(antennas.ramps > 0, "the joint set off repeatedly");
        assert_eq!(
            antennas.ramps, antennas.ramps_capped,
            "every ramp reached its speed inside the window"
        );
        assert!(
            (antennas.ramp_p50 - 0.2).abs() < 1e-9,
            "the one period's gain, not a quarter of it: {}",
            antennas.ramp_p50
        );
        let mut report = Report::default();
        capabilities(&measured, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains(&format!("{} of them capped inside it", antennas.ramps))),
            "{:?}",
            report.measured
        );
    }

    /// A gap in the sample stream is not a period: the travel across it is two
    /// or more periods' motion, and reading it as one would report a motor
    /// faster than the machine holds.
    #[test]
    fn a_gap_in_the_stream_is_not_a_per_period_travel() {
        let model = PlantModel::from_registers(
            SHIPPED_PROFILES.antennas.1,
            SHIPPED_PROFILES.antennas.0,
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
