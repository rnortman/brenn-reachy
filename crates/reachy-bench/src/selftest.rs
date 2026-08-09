//! The read-only self-test registry: what it checks, what a run says, and the
//! record it leaves behind.
//!
//! Every case here reads and nothing here writes: pings, register sweeps, the
//! supply rail, the health bytes, where the platform is resting, and what a limp
//! servo's goal register says while it rests there. No torque, no motion,
//! nothing sent to a servo but a question. That is what makes the
//! registry the gate in front of every command that moves something — it can be
//! run on an unknown machine at no risk, and it is how this project brings
//! hardware up: a case asserts what we expect, and the failure is the discovery.
//!
//! Three rules the shape of this module exists to enforce:
//!
//! - **A case that did not run is a failure, not silence.** [`Outcome::NotRun`]
//!   never counts as a pass, and a case missing from a record reads as
//!   `NotRun` rather than as absent.
//! - **The registry records; a person resolves.** The datum case reads the
//!   servos' provisioned homing offsets and writes down what it saw. That a
//!   person has read the evidence is recorded separately, in the bench
//!   configuration, with a note saying where it came from.
//! - **The record is evidence, not memory.** It says what was observed and
//!   when. Nothing reads it to find out what state the machine is in now —
//!   arming re-verifies every one of these facts against the hardware on every
//!   run.

use std::fmt;
use std::path::Path;

use anyhow::Context as _;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use dxl_proto::{HardwareError, counts_to_rad, volts_from_raw};
use reachy_bus::{Bus, BusPort, BusTiming, RawValue, ServoMap, reg_for, with_retry};
use reachy_kin::{
    FkOptions, HeadGeometry, below_limit, outside_limit, rest_head_pose, stow_head_pose,
};
use reachy_motion::{
    ArmRecord, EXPECTED_MODELS, JointId, JointVector, ProvisionExpect, ProvisionTable, RegId,
    RegValue, VENDOR_HOMING_OFFSETS,
};

use crate::config::{BenchConfig, ConfigError, DatumSetting, positive};

/// What a case decided.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Outcome {
    /// The case ran and what it asserted held.
    Pass,
    /// The case ran and what it asserted did not hold.
    Fail,
    /// The case did not run. Counts as a failure everywhere: a registry that
    /// stopped early has established nothing about the cases after the one that
    /// stopped it.
    #[default]
    NotRun,
}

impl Outcome {
    /// Whether this outcome admits going further.
    #[must_use]
    pub fn passed(self) -> bool {
        matches!(self, Self::Pass)
    }
}

impl fmt::Display for Outcome {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Pass => f.write_str("pass"),
            Self::Fail => f.write_str("FAIL"),
            Self::NotRun => f.write_str("not run"),
        }
    }
}

/// The registry's cases, in run order.
///
/// The order is the dependency order: nothing is read before the port opens,
/// nothing is asked of a servo that did not answer a ping, and the resting
/// pose's clearance is computed only after the offsets that make its counts mean
/// anything have been checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Case {
    /// The serial device opens at the configured rate.
    PortOpen,
    /// Every configured servo answers a ping. Nothing outside the roster is
    /// probed, so this establishes that the nine are there and not that they
    /// are the only ones.
    Presence,
    /// Every servo reports the model number this platform's servos report.
    Identity,
    /// The provisioned setup registers hold what the configuration says.
    ProvisionSweep,
    /// The supply rail is up on all nine servos.
    Voltage,
    /// Nothing is latched in a hardware error status byte.
    Health,
    /// Where the platform is resting, recorded.
    RestPose,
    /// Every limp servo reports its goal as its present position, which is what
    /// makes enabling torque before writing a goal safe.
    GoalShadow,
    /// The provisioned homing offsets are the vendor's, so a converted count is
    /// the model's crank angle.
    Datum,
    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations.
    RestMargins,
}

impl Case {
    /// Every case, in run order.
    pub const ALL: [Self; 10] = [
        Self::PortOpen,
        Self::Presence,
        Self::Identity,
        Self::ProvisionSweep,
        Self::Voltage,
        Self::Health,
        Self::RestPose,
        Self::GoalShadow,
        Self::Datum,
        Self::RestMargins,
    ];

    /// The case's name, as it appears in the record and in a run's output.
    #[must_use]
    pub fn slug(self) -> &'static str {
        match self {
            Self::PortOpen => "port-open",
            Self::Presence => "presence",
            Self::Identity => "identity",
            Self::ProvisionSweep => "provision-sweep",
            Self::Voltage => "voltage",
            Self::Health => "health",
            Self::RestPose => "rest-pose",
            Self::GoalShadow => "goal-shadow",
            Self::Datum => "datum",
            Self::RestMargins => "rest-margins",
        }
    }
}

impl fmt::Display for Case {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.slug())
    }
}

/// One case's verdict and the line it prints.
///
/// The detail is the case's own account of what it saw — the readings, the
/// servo that disagreed, the value that was not what was expected. It is
/// carried whether the case passed or failed, because a passing sweep's numbers
/// are the record that makes the next failure legible.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseResult {
    /// Which case.
    pub case: Case,
    /// What it decided.
    pub outcome: Outcome,
    /// What it saw, in one line.
    pub detail: String,
}

impl CaseResult {
    /// A case that ran and held.
    #[must_use]
    pub fn pass(case: Case, detail: impl Into<String>) -> Self {
        Self {
            case,
            outcome: Outcome::Pass,
            detail: detail.into(),
        }
    }

    /// A case that ran and did not hold.
    #[must_use]
    pub fn fail(case: Case, detail: impl Into<String>) -> Self {
        Self {
            case,
            outcome: Outcome::Fail,
            detail: detail.into(),
        }
    }

    /// A case that did not run, and why.
    #[must_use]
    pub fn not_run(case: Case, detail: impl Into<String>) -> Self {
        Self {
            case,
            outcome: Outcome::NotRun,
            detail: detail.into(),
        }
    }
}

impl fmt::Display for CaseResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:<16} {:<8} {}",
            self.case.slug(),
            self.outcome.to_string(),
            self.detail
        )
    }
}

/// What a case decided in a list of verdicts, or [`Outcome::NotRun`] if the
/// list does not mention it.
///
/// A case nobody recorded counts as a failure, which is what makes a truncated
/// record refuse arming. The rule has one spelling so a run in progress and the
/// record it became can never judge the same verdicts differently.
///
/// A case answered more than once reads as its first non-passing answer. A run
/// pushes each case once, so this only bears on a record edited or concatenated
/// on disk between the run that wrote it and a consumer that parses it —
/// [`SelftestRecord::parse`] refuses those outright, and taking the worse
/// answer here means the two checks cannot disagree.
fn outcome_of(cases: &[CaseResult], case: Case) -> Outcome {
    let mut verdict = None;
    for result in cases.iter().filter(|result| result.case == case) {
        if !result.outcome.passed() {
            return result.outcome;
        }
        verdict = verdict.or(Some(result.outcome));
    }
    verdict.unwrap_or(Outcome::NotRun)
}

/// A case a list of verdicts answers more than once, if there is one.
///
/// Cases are walked in run order so the refusal names the earliest, which is
/// where a person reading the file starts.
fn duplicate_case(cases: &[CaseResult]) -> Option<Case> {
    Case::ALL
        .into_iter()
        .find(|case| cases.iter().filter(|result| result.case == *case).count() > 1)
}

/// The first case in run order that is not a pass, with what the verdicts say
/// about it.
fn first_not_passed(cases: &[CaseResult]) -> Option<(Case, Outcome)> {
    Case::ALL.into_iter().find_map(|case| {
        let outcome = outcome_of(cases, case);
        (!outcome.passed()).then_some((case, outcome))
    })
}

/// A run in progress, and the record it becomes.
///
/// Cases are pushed in run order. A case never pushed is reported as
/// [`Outcome::NotRun`], so a run that stopped at the presence sweep prints a
/// line per case and fails, rather than printing two and looking short.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    results: Vec<CaseResult>,
    rest_counts: Option<[i32; JointId::COUNT]>,
    models: Option<[u16; JointId::COUNT]>,
    datum: Option<DatumRecord>,
}

impl Report {
    /// A run with nothing decided yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a case's verdict.
    pub fn push(&mut self, result: CaseResult) {
        self.results.push(result);
    }

    /// The verdicts so far, in the order they were reached.
    #[must_use]
    pub fn results(&self) -> &[CaseResult] {
        &self.results
    }

    /// What a case decided, or [`Outcome::NotRun`] if it has not.
    #[must_use]
    pub fn outcome(&self, case: Case) -> Outcome {
        outcome_of(&self.results, case)
    }

    /// Whether every case in the registry passed.
    #[must_use]
    pub fn all_passed(&self) -> bool {
        first_not_passed(&self.results).is_none()
    }

    /// The resting position readings, as counts in bus order.
    pub fn set_rest_counts(&mut self, counts: [i32; JointId::COUNT]) {
        self.rest_counts = Some(counts);
    }

    /// The model numbers, in bus order.
    pub fn set_models(&mut self, models: [u16; JointId::COUNT]) {
        self.models = Some(models);
    }

    /// What the datum case read.
    pub fn set_datum(&mut self, datum: DatumRecord) {
        self.datum = Some(datum);
    }

    /// The record this run leaves behind, timestamped by the caller.
    #[must_use]
    pub fn into_record(self, taken_at_unix: u64) -> SelftestRecord {
        SelftestRecord {
            taken_at_unix,
            rest_counts: self.rest_counts,
            models: self.models,
            datum: self.datum,
            cases: self.results,
        }
    }
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for case in Case::ALL {
            match self.results.iter().find(|result| result.case == case) {
                Some(result) => writeln!(f, "{result}")?,
                None => writeln!(f, "{}", CaseResult::not_run(case, "did not run"))?,
            }
        }
        Ok(())
    }
}

/// What the datum case saw and what it establishes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatumRecord {
    /// The datum the offsets establish, written down only when they matched.
    pub crank_datum: DatumSetting,
    /// The nine homing offset registers as read, counts in bus order — the
    /// evidence the datum rests on.
    pub homing_offsets: [i32; JointId::COUNT],
}

impl DatumRecord {
    /// Record the datum the observed offsets establish.
    #[must_use]
    pub fn new(crank_datum: DatumSetting, homing_offsets: [i32; JointId::COUNT]) -> Self {
        Self {
            crank_datum,
            homing_offsets,
        }
    }
}

/// What a self-test run observed, and when.
///
/// Written beside the bench configuration. It is evidence about a moment, never
/// a cache of machine state: every fact in it is re-established against the
/// hardware by commissioning before anything moves.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SelftestRecord {
    /// When the run finished, seconds since the Unix epoch. Seconds because
    /// nothing here parses dates; a reader converts.
    pub taken_at_unix: u64,
    /// The resting position readings, counts in bus order.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_counts: Option<[i32; JointId::COUNT]>,
    /// The model numbers, bus order — recorded for human review before any
    /// expected value is baked into the identity case.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub models: Option<[u16; JointId::COUNT]>,
    /// What the datum case read, present only when the offsets matched.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub datum: Option<DatumRecord>,
    /// Every case's verdict.
    #[serde(default)]
    pub cases: Vec<CaseResult>,
}

/// Why a record does not admit a command that moves something.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RecordRefusal {
    /// A case that is not a pass, the missing ones included.
    #[error(
        "the self-test case `{case}` is recorded as {outcome}: every case must pass before anything is commanded"
    )]
    CaseNotPassed {
        /// The case.
        case: Case,
        /// What the record says about it.
        outcome: Outcome,
    },
}

impl SelftestRecord {
    /// What the record says about a case; a case it does not mention did not
    /// run.
    #[must_use]
    pub fn outcome(&self, case: Case) -> Outcome {
        outcome_of(&self.cases, case)
    }

    /// Whether every case in this record passed, and which one did not.
    ///
    /// A verdict for a reader, not a gate: nothing in this workspace conditions
    /// arming or commanding on a record. The datum is one of the cases, and it
    /// passes only when all nine homing offsets are the vendor's; that a person
    /// reviewed that evidence is the configuration's `[datum]` table, which is
    /// checked at load because every converted angle rests on it.
    pub fn every_case_passed(&self) -> Result<(), RecordRefusal> {
        if let Some((case, outcome)) = first_not_passed(&self.cases) {
            return Err(RecordRefusal::CaseNotPassed { case, outcome });
        }
        Ok(())
    }

    /// The record as TOML.
    pub fn render(&self) -> anyhow::Result<String> {
        toml::to_string_pretty(self).context("rendering the self-test record")
    }

    /// Write the record beside the configuration.
    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        std::fs::write(path, self.render()?)
            .with_context(|| format!("writing the self-test record to {}", path.display()))
    }

    /// Read a record from TOML text.
    ///
    /// A record that answers a case twice is refused rather than read past. The
    /// file sits on disk between the run that wrote it and the arm that reads
    /// it, and two rows for one case — a hand edit, a merge, two runs
    /// concatenated — say two things about one question. Which of them is the
    /// evidence is a person's call.
    pub fn parse(text: &str) -> anyhow::Result<Self> {
        let record: Self = toml::from_str(text)?;
        if let Some(case) = duplicate_case(&record.cases) {
            anyhow::bail!(
                "the self-test record answers the case `{case}` more than once: re-run the \
                 self-test rather than reading one of the answers"
            );
        }
        Ok(record)
    }

    /// Read the record beside the configuration.
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("reading the self-test record at {}", path.display()))?;
        Self::parse(&text)
            .with_context(|| format!("parsing the self-test record at {}", path.display()))
    }
}

/// The clearance the resting pose has to leave from the linkage's singular
/// configurations, metres.
///
/// Reviewed, not invented. Across settle sessions on this unit the smallest
/// clearance measured was 3.02 mm — after the head had been deliberately
/// repositioned and firmly re-seated; an earlier undisturbed settle measured
/// 6.11 mm — and this floor sits at half of the worst of them.
///
/// Deliberately loose, because of what it is for: this is an
/// is-the-machine-assembled-sanely guard, and a machine resting an order of
/// magnitude tighter than any settle has ever produced is one to look at rather
/// than to arm. It is not the fence a move is held to. What governs motion off a
/// tight rest is the present clearance taken as a baseline, in the envelope's
/// own margin check, and that runs on every commanded pose.
///
/// `None` puts the case back to recording what it measures without judging it,
/// which is where a fresh unit with no settle history of its own starts.
pub const REST_MARGIN_FLOOR_M: Option<f64> = Some(0.0015);

/// The clearance case's verdict on a measured minimum margin.
///
/// A free function rather than a match inside the case so the comparison that
/// decides pass from fail is reachable whatever [`REST_MARGIN_FLOOR_M`] holds.
/// The floor is what the first reviewed hardware run fills in, and the arm that
/// then executes for the first time must not be one nothing has ever run: a
/// polarity the wrong way round would report a pass at a clearance *under* the
/// floor, admitting arming on a resting pose close to a singular configuration.
///
/// A margin exactly at the floor passes — the floor is the clearance that has to
/// be left, not the one that has to be beaten.
#[must_use]
pub fn margin_verdict(min_margin: f64, floor: Option<f64>, detail: &str) -> CaseResult {
    match floor {
        None => CaseResult::not_run(
            Case::RestMargins,
            format!("{detail} — recorded; no reviewed clearance floor exists to assert against"),
        ),
        Some(floor) if below_limit(min_margin, floor) => CaseResult::fail(
            Case::RestMargins,
            format!("{detail}, under the {:.4} mm floor", floor * 1000.0),
        ),
        Some(floor) => CaseResult::pass(
            Case::RestMargins,
            format!("{detail}, clear of the {:.4} mm floor", floor * 1000.0),
        ),
    }
}

/// Everything the read-only registry needs that is not the machine.
///
/// Built from the configuration file *before* it resolves, because resolving
/// refuses a file with no reviewed `[datum]` record and this registry is what
/// produces the evidence that record is written from. Nothing here is a value
/// the registry could send: the only wire traffic it produces is pings and
/// register reads.
#[derive(Clone, Debug)]
pub struct Registry {
    device: String,
    ids: [u8; JointId::COUNT],
    timing: BusTiming,
    map: ServoMap,
    expected: ProvisionTable,
    geom: HeadGeometry,
    fk: FkOptions,
    min_arm_voltage: f64,
    goal_shadow_tolerance: f64,
}

impl Registry {
    /// The registry a configuration describes.
    ///
    /// Only the tables the read-only half needs are converted, so a file with no
    /// `[datum]` table — every file, before the first run — still produces a
    /// runnable registry.
    pub fn from_config(cfg: &BenchConfig) -> Result<Self, ConfigError> {
        let ids = cfg.servo_ids()?;
        Ok(Self {
            device: cfg.bus.device.clone(),
            ids,
            timing: cfg.bus_timing()?,
            map: ServoMap::new(ids),
            expected: cfg.provision_table(),
            geom: HeadGeometry::default(),
            fk: FkOptions::default(),
            // Checked here rather than inherited: resolving the configuration is
            // what normally runs this gate, and the registry deliberately does
            // not resolve — it must run before the datum record exists. Without
            // it a non-positive floor would let the voltage case pass a dead
            // rail and write that pass into the record a person reviews.
            min_arm_voltage: positive("arm.min_arm_voltage", cfg.arm.min_arm_voltage)?,
            // Checked here for the same reason and against the same risk: an
            // infinite or non-finite tolerance would pass a machine whose goal
            // registers say anything at all, and write that pass into the record
            // the arm gate reads.
            goal_shadow_tolerance: positive(
                "arm.goal_shadow_tolerance_deg",
                cfg.arm.goal_shadow_tolerance_deg,
            )?
            .to_radians(),
        })
    }

    /// Run the registry against `port`, filling `report`.
    ///
    /// The port arrives as a result rather than already open so that the port
    /// case is one of the nine rather than something that happened before the
    /// run started. A case that cannot run leaves its verdict at
    /// [`Outcome::NotRun`], which prints and fails.
    pub fn run<P, E>(&self, port: Result<P, E>, report: &mut Report)
    where
        P: BusPort,
        E: fmt::Display,
    {
        let port = match port {
            Ok(port) => {
                report.push(CaseResult::pass(
                    Case::PortOpen,
                    format!("{} open at {} baud", self.device, self.timing.baud),
                ));
                port
            }
            Err(error) => {
                report.push(CaseResult::fail(
                    Case::PortOpen,
                    format!("{}: {error}", self.device),
                ));
                return;
            }
        };

        let mut bus = Bus::new(port, self.timing);
        if !self.presence(&mut bus, report) {
            return;
        }
        self.identity(&mut bus, report);
        self.provision(&mut bus, report);
        self.voltage(&mut bus, report);
        self.health(&mut bus, report);
        let Some(counts) = self.rest_pose(&mut bus, report) else {
            return;
        };
        self.goal_shadow(&mut bus, report);
        self.datum(&mut bus, report);
        self.rest_margins(&counts, report);
    }

    /// Ping every configured servo, naming every one that does not answer.
    ///
    /// All nine are asked before the case decides, so one silent servo does not
    /// hide the next. Nothing is diagnosed from the pattern of silence: nine
    /// absent servos are reported as nine absent servos.
    fn presence<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) -> bool {
        let mut silent = Vec::new();
        let mut firmware = Vec::new();
        for id in self.ids {
            match with_retry(bus, |bus| bus.ping(id)) {
                Ok(info) => firmware.push(format!("{id}:v{}", info.firmware)),
                Err(error) => silent.push(format!("{id} ({error})")),
            }
        }
        if silent.is_empty() {
            report.push(CaseResult::pass(
                Case::Presence,
                format!(
                    "every configured servo answered, firmware {}; no address outside the \
                     configured roster was probed, and no rate other than {} was tried",
                    firmware.join(" "),
                    self.timing.baud
                ),
            ));
            true
        } else {
            report.push(CaseResult::fail(
                Case::Presence,
                format!("silent: {}", silent.join(", ")),
            ));
            false
        }
    }

    /// The model numbers, against the ones this platform's servos report.
    ///
    /// Every reading is recorded either way; a servo answering with another
    /// number is a servo of another kind at a roster address, and the case names
    /// it. Firmware versions are recorded by the presence case and asserted
    /// nowhere: a vendor update moves them and says nothing about the hardware.
    fn identity<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let models = match self.sweep_u16(bus, RegId::ModelNumber) {
            Ok(models) => models,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Identity, detail));
                return;
            }
        };
        report.set_models(models);

        let wrong: Vec<String> = self
            .ids
            .iter()
            .zip(models.iter().zip(EXPECTED_MODELS.iter()))
            .filter(|(_, (model, expected))| model != expected)
            .map(|(id, (model, expected))| format!("servo {id}: expected {expected}, read {model}"))
            .collect();
        let detail = format!("models {models:?}");
        if wrong.is_empty() {
            report.push(CaseResult::pass(Case::Identity, detail));
        } else {
            report.push(CaseResult::fail(
                Case::Identity,
                format!("{detail} — {}", wrong.join("; ")),
            ));
        }
    }

    /// Read every provisioned register the configuration names and compare the
    /// ones it claims a value for.
    ///
    /// Registers walked outer, servos inner, so a register's nine readings print
    /// side by side — which is the shape a person reads a provisioning
    /// disagreement out of.
    fn provision<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let mut checked = 0usize;
        let mut recorded = 0usize;
        let mut wrong = Vec::new();
        let mut readings = Vec::new();

        for reg in RegId::ALL {
            let Some(column) = ProvisionTable::column(reg) else {
                continue;
            };
            let mut row_values = Vec::with_capacity(JointId::COUNT);
            let mut read_any = false;
            for (row, id) in self.ids.iter().enumerate() {
                let expect = self.expected.at(row, column).unwrap_or_default();
                if matches!(expect, ProvisionExpect::Skip) {
                    row_values.push("-".to_string());
                    continue;
                }
                let value = match self.read_value(bus, row, reg) {
                    Ok(value) => value,
                    Err(detail) => {
                        report.push(CaseResult::fail(Case::ProvisionSweep, detail));
                        return;
                    }
                };
                read_any = true;
                row_values.push(value.to_string());
                match expect {
                    ProvisionExpect::Check(expected) => {
                        checked += 1;
                        if value != expected {
                            wrong.push(format!(
                                "servo {id} {reg}: expected {expected}, read {value}"
                            ));
                        }
                    }
                    ProvisionExpect::Record => recorded += 1,
                    ProvisionExpect::Skip => {}
                }
            }
            if read_any {
                readings.push(format!("{reg} [{}]", row_values.join(" ")));
            }
        }

        let summary = format!(
            "{checked} checked, {recorded} recorded: {}",
            readings.join("; ")
        );
        if wrong.is_empty() {
            report.push(CaseResult::pass(Case::ProvisionSweep, summary));
        } else {
            report.push(CaseResult::fail(
                Case::ProvisionSweep,
                format!("{}; {summary}", wrong.join("; ")),
            ));
        }
    }

    /// The supply rail on all nine servos, against the floor arming refuses to
    /// proceed below.
    fn voltage<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let raw = match self.sweep_u16(bus, RegId::PresentInputVoltage) {
            Ok(raw) => raw,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Voltage, detail));
                return;
            }
        };
        let volts = raw.map(volts_from_raw);
        let low: Vec<String> = self
            .ids
            .iter()
            .zip(volts.iter())
            .filter(|(_, reading)| below_limit(**reading, self.min_arm_voltage))
            .map(|(id, reading)| format!("{id} at {reading:.1} V"))
            .collect();
        let detail = format!(
            "{volts:.1?} V against a {:.1} V floor",
            self.min_arm_voltage
        );
        if low.is_empty() {
            report.push(CaseResult::pass(Case::Voltage, detail));
        } else {
            report.push(CaseResult::fail(
                Case::Voltage,
                format!("below the floor: {}; {detail}", low.join(", ")),
            ));
        }
    }

    /// The latched hardware-error bytes.
    ///
    /// An input-voltage bit on its own passes and is still reported: it latches
    /// on a supply dip that has since recovered, and suppressing it would hide
    /// the one bit a bench supply routinely sets.
    fn health<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let bytes = match self.sweep_u8(bus, RegId::HardwareErrorStatus) {
            Ok(bytes) => bytes,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Health, detail));
                return;
            }
        };
        let unhealthy: Vec<String> = self
            .ids
            .iter()
            .zip(bytes.iter())
            .filter(|(_, bits)| !HardwareError(**bits).healthy_or_voltage_only())
            .map(|(id, bits)| format!("{id} = {bits:#04x}"))
            .collect();
        // Formatted by hand rather than through the slice's own debug: the
        // hexadecimal debug format breaks a byte array across lines, and a case
        // is one line.
        let listed: Vec<String> = bytes.iter().map(|bits| format!("{bits:#04x}")).collect();
        let detail = format!("bytes [{}]", listed.join(" "));
        if unhealthy.is_empty() {
            report.push(CaseResult::pass(Case::Health, detail));
        } else {
            report.push(CaseResult::fail(
                Case::Health,
                format!("latched: {}; {detail}", unhealthy.join(", ")),
            ));
        }
    }

    /// Where the platform is resting, as counts.
    ///
    /// Counts rather than angles: a count is what the servo said, and the
    /// conversion to an angle is only the model's angle once the datum case has
    /// confirmed the offsets it rests on.
    fn rest_pose<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        report: &mut Report,
    ) -> Option<[i32; JointId::COUNT]> {
        let counts = match self.sweep_i32(bus, RegId::PresentPosition) {
            Ok(counts) => counts,
            Err(detail) => {
                report.push(CaseResult::fail(Case::RestPose, detail));
                return None;
            }
        };
        report.set_rest_counts(counts);
        let direct_deg = counts.map(|count| counts_to_rad(count).to_degrees());
        report.push(CaseResult::pass(
            Case::RestPose,
            format!("counts {counts:?}, unshifted {direct_deg:.3?} deg"),
        ));
        Some(counts)
    }

    /// Every limp servo's Goal Position register against the position it is
    /// reporting.
    ///
    /// This is the precondition the whole torque-on path rests on. A servo with
    /// torque off reports its goal as its present position, so at the instant
    /// torque comes on the target it starts holding is where it already stands
    /// and an enable cannot slam. Engaging writes the measured position to every
    /// goal register first as insurance, but that write lands nowhere on a
    /// mirroring servo — the mirroring is the safety property and the write is
    /// the belt beside it. A servo answering otherwise is a machine this project
    /// has no safety argument for, and it says so here — with no torque on and
    /// nothing written, which is why the question is asked in the read-only half
    /// rather than discovered mid-engage.
    ///
    /// A servo found holding torque is exempt: its goal is a target it really is
    /// holding, and the gap to its present position is the sag of a loaded
    /// servo. Reported, never judged.
    fn goal_shadow<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let torque = match self.sweep_u8(bus, RegId::TorqueEnable) {
            Ok(torque) => torque,
            Err(detail) => {
                report.push(CaseResult::fail(Case::GoalShadow, detail));
                return;
            }
        };
        // Read here rather than taken from the resting-pose case: the comparison
        // is between two registers of one servo, and a present position read a
        // sweep earlier could have moved under a hand in between.
        let present = match self.sweep_i32(bus, RegId::PresentPosition) {
            Ok(present) => present,
            Err(detail) => {
                report.push(CaseResult::fail(Case::GoalShadow, detail));
                return;
            }
        };
        let goal = match self.sweep_i32(bus, RegId::GoalPosition) {
            Ok(goal) => goal,
            Err(detail) => {
                report.push(CaseResult::fail(Case::GoalShadow, detail));
                return;
            }
        };

        let mut readings = Vec::with_capacity(JointId::COUNT);
        let mut wrong = Vec::new();
        let mut holding = 0usize;
        for (row, id) in self.ids.iter().enumerate() {
            let (Ok(goal_rad), Ok(present_rad)) = (
                self.map.present_rad(row, goal[row]),
                self.map.present_rad(row, present[row]),
            ) else {
                report.push(CaseResult::fail(
                    Case::GoalShadow,
                    format!("servo {id}: bus row {row} is not one of the nine joints"),
                ));
                return;
            };
            let gap = goal_rad - present_rad;
            let torqued = torque[row] != 0;
            readings.push(format!(
                "{id}: torque {}, goal {} present {} ({:+.3} deg)",
                if torqued { "on" } else { "off" },
                goal[row],
                present[row],
                gap.to_degrees()
            ));
            if torqued {
                holding += 1;
            } else if outside_limit(gap.abs(), self.goal_shadow_tolerance) {
                wrong.push(format!(
                    "servo {id}: goal {} sits {:+.3} deg off the present {} it should shadow",
                    goal[row],
                    gap.to_degrees(),
                    present[row]
                ));
            }
        }

        let detail = format!(
            "{} limp, {holding} holding torque and exempt, against a {:.3} deg tolerance: {}",
            JointId::COUNT - holding,
            self.goal_shadow_tolerance.to_degrees(),
            readings.join("; ")
        );
        if wrong.is_empty() {
            report.push(CaseResult::pass(Case::GoalShadow, detail));
        } else {
            report.push(CaseResult::fail(
                Case::GoalShadow,
                format!("{}; {detail}", wrong.join("; ")),
            ));
        }
    }

    /// The nine provisioned homing offsets against the vendor's own constant.
    ///
    /// This is what the datum rests on: each servo applies its offset before
    /// reporting a position, so a converted count is the model's crank angle
    /// exactly when these nine registers hold what the vendor wrote. A servo
    /// answering otherwise fails the case by name — the repair is the vendor's
    /// provisioning tool, and no host-side correction for it exists anywhere in
    /// this workspace.
    fn datum<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let offsets = match self.sweep_i32(bus, RegId::HomingOffset) {
            Ok(offsets) => offsets,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Datum, detail));
                return;
            }
        };
        let wrong: Vec<String> = self
            .ids
            .iter()
            .zip(offsets.iter().zip(VENDOR_HOMING_OFFSETS.iter()))
            .filter(|(_, (read, expected))| read != expected)
            .map(|(id, (read, expected))| format!("servo {id}: expected {expected}, read {read}"))
            .collect();
        let detail = format!("offsets {offsets:?}");
        if wrong.is_empty() {
            report.set_datum(DatumRecord::new(DatumSetting::Direct, offsets));
            report.push(CaseResult::pass(
                Case::Datum,
                format!(
                    "{detail} — the vendor constant, so the datum is {}",
                    DatumSetting::Direct
                ),
            ));
        } else {
            report.push(CaseResult::fail(
                Case::Datum,
                format!("{}; {detail}", wrong.join("; ")),
            ));
        }
    }

    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations.
    fn rest_margins(&self, counts: &[i32; JointId::COUNT], report: &mut Report) {
        let mut joints = JointVector::default();
        // The joint layout is asked for rather than restated: each reading is
        // filed against the joint that bus row belongs to.
        for (row, (id, count)) in JointId::ALL.into_iter().zip(counts.iter()).enumerate() {
            let angle = match self.map.present_rad(row, *count) {
                Ok(angle) => angle,
                Err(error) => {
                    report.push(CaseResult::fail(Case::RestMargins, error.to_string()));
                    return;
                }
            };
            joints.set(id, angle);
        }

        let record = match ArmRecord::solve(
            &self.geom,
            &self.fk,
            &joints,
            &[rest_head_pose(), stow_head_pose()],
        ) {
            Ok(record) => record,
            Err(error) => {
                report.push(CaseResult::fail(
                    Case::RestMargins,
                    format!("the resting angles close no plausible pose: {error}"),
                ));
                return;
            }
        };

        let margins_mm = record.margins.map(|margin| margin * 1000.0);
        let height = record.head_pose_body.translation.z;
        let detail = format!("margins {margins_mm:.4?} mm, head at {height:.4} m");
        report.push(margin_verdict(
            record.min_margin,
            REST_MARGIN_FLOOR_M,
            &detail,
        ));
    }

    /// One register from one servo, as its engineering value.
    fn read_value<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        row: usize,
        reg: RegId,
    ) -> Result<RegValue, String> {
        let raw = self.read_raw(bus, row, reg)?;
        let id = self.ids[row];
        self.map
            .decode_value(row, reg, &raw)
            .map_err(|error| format!("servo {id} {reg}: {error}"))
    }

    /// One register from one servo, as the bytes it holds.
    fn read_raw<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        row: usize,
        reg: RegId,
    ) -> Result<RawValue, String> {
        let id = self.ids[row];
        let entry = reg_for(reg);
        with_retry(bus, |bus| bus.read_reg(id, entry))
            .map_err(|error| format!("servo {id} {reg}: {error}"))
    }

    /// One register from all nine servos, taken apart by `extract`.
    ///
    /// The sweep stops at the first servo that will not answer or answers in a
    /// width the register does not read as, so a run says which servo and which
    /// register rather than how many failed.
    fn sweep<P: BusPort, T: Copy + Default>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
        extract: impl Fn(&RawValue) -> Option<T>,
    ) -> Result<[T; JointId::COUNT], String> {
        let mut out = [T::default(); JointId::COUNT];
        for (row, slot) in out.iter_mut().enumerate() {
            let raw = self.read_raw(bus, row, reg)?;
            *slot = extract(&raw).ok_or_else(|| self.width_detail(row, reg, &raw))?;
        }
        Ok(out)
    }

    /// One one-byte register from all nine servos.
    fn sweep_u8<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
    ) -> Result<[u8; JointId::COUNT], String> {
        self.sweep(bus, reg, RawValue::u8)
    }

    /// One two-byte register from all nine servos.
    fn sweep_u16<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
    ) -> Result<[u16; JointId::COUNT], String> {
        self.sweep(bus, reg, RawValue::u16)
    }

    /// One four-byte signed register from all nine servos.
    fn sweep_i32<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
    ) -> Result<[i32; JointId::COUNT], String> {
        self.sweep(bus, reg, RawValue::i32)
    }

    /// A reply the register's width does not fit. Unreachable while the control
    /// table and the transaction layer's length check agree; reported rather
    /// than assumed away because the case's job is saying what it saw.
    fn width_detail(&self, row: usize, reg: RegId, raw: &RawValue) -> String {
        format!(
            "servo {} {reg}: {} bytes are not the width this register reads as",
            self.ids[row],
            raw.len()
        )
    }
}

/// The wall clock, seconds since the Unix epoch.
///
/// A clock before the epoch reads as the epoch rather than as a refusal: the
/// timestamp is a note to a person, and nothing decides anything from it.
#[must_use]
pub fn now_unix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |since| since.as_secs())
}

#[cfg(test)]
mod runner_tests {
    use dxl_proto::frame::{INST_PING, INST_READ};

    use super::*;
    use crate::testutil::{FakeMachine, Spy, machine_at, rest_legs, stow_legs, undatumed_config};

    /// A run of the registry against a machine, with the port already open, and
    /// every instruction that crossed the wire.
    fn run(cfg: &BenchConfig, machine: FakeMachine) -> (Report, Vec<(u8, u8)>) {
        let (report, instructions, _) = run_watched(cfg, machine);
        (report, instructions)
    }

    /// What a watched run leaves behind: the verdicts, every instruction that
    /// crossed the wire as (servo, instruction), and every unicast read as
    /// (servo, register address).
    type Watched = (Report, Vec<(u8, u8)>, Vec<(u8, u16)>);

    /// The same run, carrying the register each unicast read asked for as well
    /// as the instructions — which is the only record of *which* register a case
    /// put on the wire, and how often.
    fn run_watched(cfg: &BenchConfig, machine: FakeMachine) -> Watched {
        let registry = Registry::from_config(cfg).expect("the configuration converts");
        let spy = Spy::new(machine);
        let log = spy.log();
        let reads = spy.reads();
        let mut report = Report::new();
        registry.run(Ok::<Spy, String>(spy), &mut report);
        let instructions = log.borrow().clone();
        let asked = reads.borrow().clone();
        (report, instructions, asked)
    }

    /// A machine holding what it should, resting at the stow pose, passes every
    /// case the registry has — the clearance among them, since the floor is a
    /// reviewed number and a correct machine clears it.
    #[test]
    fn a_correct_machine_passes_every_case() {
        let cfg = undatumed_config();
        let machine = machine_at(&cfg, &stow_legs());
        let (report, _) = run(&cfg, machine);

        for case in Case::ALL {
            assert_eq!(
                report.outcome(case),
                Outcome::Pass,
                "{case}: {}",
                report
                    .results()
                    .iter()
                    .find(|result| result.case == case)
                    .map_or_else(|| "no verdict".to_string(), ToString::to_string)
            );
        }
        assert!(report.all_passed(), "every case passed");

        let record = report.into_record(1);
        assert!(record.rest_counts.is_some());
        assert_eq!(record.models, Some(EXPECTED_MODELS));
        let datum = record.datum.expect("the datum case recorded");
        assert_eq!(datum.crank_datum, DatumSetting::Direct);
        assert_eq!(datum.homing_offsets, VENDOR_HOMING_OFFSETS);
    }

    /// The registry pings and reads, addresses nothing but its own roster, and
    /// does nothing else. This is the property that makes it safe to run on an
    /// unknown machine — and the presence case prints it as a statement of
    /// fact, so both halves are asserted here rather than only the instruction.
    #[test]
    fn the_registry_only_ever_pings_and_reads_its_own_roster() {
        let cfg = undatumed_config();
        let machine = machine_at(&cfg, &stow_legs());
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let (_, instructions) = run(&cfg, machine);

        assert!(!instructions.is_empty());
        let mut seen = Vec::new();
        for (id, instruction) in &instructions {
            assert!(
                *instruction == INST_PING || *instruction == INST_READ,
                "servo {id} was sent instruction {instruction:#04x}"
            );
            assert_ne!(
                *id,
                dxl_proto::BROADCAST_ID,
                "a broadcast reaches servos nobody configured"
            );
            assert!(ids.contains(id), "address {id} is not on the roster");
            if !seen.contains(id) {
                seen.push(*id);
            }
        }
        seen.sort_unstable();
        assert_eq!(seen, ids.to_vec(), "every configured servo, and only those");
    }

    /// A clearance floor the configuration cannot stand behind refuses before
    /// the registry is built, rather than being carried into the voltage case.
    ///
    /// The registry does not resolve the configuration — it has to run before a
    /// datum exists — so the positivity gate every other consumer gets from
    /// `resolve` has to be run here too. Without it a negative floor passes a
    /// dead rail and writes that pass into the record a person reviews.
    #[test]
    fn an_arming_floor_that_is_not_a_voltage_refuses_the_registry() {
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY] {
            let mut cfg = undatumed_config();
            cfg.arm.min_arm_voltage = bad;
            let Err(refusal) = Registry::from_config(&cfg) else {
                panic!("{bad} is not a supply floor, and a registry was built on it");
            };
            // Compared by key rather than by whole value: a `NaN` payload is
            // never equal to itself, which is the reason it is refused.
            let ConfigError::NotPositive { key, value } = refusal else {
                panic!("{bad}: expected a positivity refusal, got {refusal}");
            };
            assert_eq!(key, "arm.min_arm_voltage");
            assert_eq!(value.to_bits(), bad.to_bits(), "{bad}");
        }
    }

    /// A port that does not open fails the first case and runs nothing after it.
    #[test]
    fn a_port_that_does_not_open_stops_the_run_at_the_first_case() {
        let cfg = undatumed_config();
        let registry = Registry::from_config(&cfg).expect("the configuration converts");
        let mut report = Report::new();
        registry.run(Err::<FakeMachine, _>("no such device"), &mut report);

        assert_eq!(report.outcome(Case::PortOpen), Outcome::Fail);
        assert_eq!(report.results().len(), 1);
        for case in Case::ALL.iter().skip(1) {
            assert_eq!(report.outcome(*case), Outcome::NotRun);
        }
        assert!(report.to_string().contains("no such device"));
    }

    /// Every silent servo is named, and the run stops rather than asking
    /// questions of a machine that is not all there.
    #[test]
    fn silent_servos_are_all_named_and_stop_the_run() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.silent = vec![12, 17];
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Presence), Outcome::Fail);
        assert_eq!(report.outcome(Case::Identity), Outcome::NotRun);
        let printed = report.to_string();
        assert!(printed.contains("12 ("), "{printed}");
        assert!(printed.contains("17 ("), "{printed}");
    }

    /// A provisioned register that does not hold what the configuration says
    /// fails the sweep, names the servo, the register, and both values — and the
    /// sweep still finishes, so one disagreement does not hide the next.
    #[test]
    fn a_provisioned_register_that_disagrees_is_named_with_both_values() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // Operating mode 0 on one servo: the reading that voids the servo-side
        // position envelope.
        machine.set(13, reg_for(RegId::OperatingMode), &[0]);
        machine.set(16, reg_for(RegId::TemperatureLimit), &[95]);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 13 operating mode"), "{printed}");
        assert!(printed.contains("servo 16 temperature limit"), "{printed}");
        // The cases after it still ran: a provisioning disagreement is evidence,
        // not a reason to stop reading.
        assert_eq!(report.outcome(Case::Voltage), Outcome::Pass);
        assert_eq!(report.outcome(Case::Health), Outcome::Pass);
    }

    /// An antenna still in single-turn position mode fails the sweep by name.
    ///
    /// This is the gate a unit meets before it is provisioned: the antennas are
    /// expected in extended position (4), which the vendor does not set, so the
    /// first self-test on a fresh unit fails here and `provision` is the answer.
    /// The seven other servos are expected in single-turn mode on the same run,
    /// so the expectation is genuinely per servo rather than one value.
    #[test]
    fn an_antenna_in_single_turn_mode_fails_the_sweep_by_name() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(17, reg_for(RegId::OperatingMode), &[3]);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 17 operating mode"), "{printed}");
        assert!(
            printed.contains('4'),
            "the expected mode is named: {printed}"
        );

        // The same fixture with the antennas as this project provisions them
        // passes, so the case above is the mode and not the fixture.
        let (report, _) = run(&cfg, machine_at(&cfg, &stow_legs()));
        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Pass);
    }

    /// A rail under the arm floor fails, and the reading is reported either way.
    #[test]
    fn a_rail_under_the_arm_floor_fails_and_says_which_servo() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            15,
            reg_for(RegId::PresentInputVoltage),
            &45u16.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Voltage), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("15 at 4.5 V"), "{printed}");
        assert!(printed.contains("11.8"), "{printed}");
    }

    /// A latched bit beyond input voltage fails; input voltage on its own passes
    /// and is still printed.
    #[test]
    fn only_bits_beyond_input_voltage_fail_health() {
        let cfg = undatumed_config();
        let legs = stow_legs();

        let mut voltage_only = machine_at(&cfg, &legs);
        voltage_only.set(11, reg_for(RegId::HardwareErrorStatus), &[0x01]);
        let (report, _) = run(&cfg, voltage_only);
        assert_eq!(report.outcome(Case::Health), Outcome::Pass);
        assert!(report.to_string().contains("0x01"), "the byte is reported");

        let mut overload = machine_at(&cfg, &legs);
        overload.set(11, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let (report, _) = run(&cfg, overload);
        assert_eq!(report.outcome(Case::Health), Outcome::Fail);
        assert!(
            report.to_string().contains("11 = 0x20"),
            "the servo is named"
        );
    }

    /// A servo answering with a model number this platform's servos do not fails
    /// the case, named with both values.
    #[test]
    fn a_servo_of_the_wrong_kind_fails_the_identity_case() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // The right antenna answering as one of the legs.
        machine.set(
            17,
            reg_for(RegId::ModelNumber),
            &EXPECTED_MODELS[1].to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Identity), Outcome::Fail);
        assert!(
            report
                .to_string()
                .contains("servo 17: expected 1190, read 1200"),
            "{report}"
        );
        // Recorded regardless: the reading is the evidence a person reviews.
        let record = report.into_record(1);
        assert_eq!(
            record.models.map(|models| models[7]),
            Some(EXPECTED_MODELS[1]),
            "the observed value is recorded even when it fails"
        );
    }

    /// One servo whose homing offset is not the vendor's fails the datum case,
    /// naming the servo and both values.
    ///
    /// The whole of the datum is that these nine registers hold what the vendor
    /// wrote. A servo that lost its offset — replaced hardware, a factory reset
    /// — reports a crank angle a quarter turn from the model's, and nothing
    /// downstream of here would notice: the counts stay inside the servo's own
    /// provisioned window, the linkage still closes, and the pose is simply not
    /// where the head is.
    #[test]
    fn a_servo_without_its_vendor_homing_offset_fails_the_datum_case() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(13, reg_for(RegId::HomingOffset), &0i32.to_le_bytes());
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Datum), Outcome::Fail);
        let printed = report.to_string();
        assert!(
            printed.contains("servo 13: expected 1024, read 0"),
            "{printed}"
        );

        let record = report.into_record(1);
        assert_eq!(
            record.datum, None,
            "a datum is written down only when the offsets establish one"
        );
        assert!(
            record.every_case_passed().is_err(),
            "a machine whose provisioning does not establish the datum arms nothing"
        );
    }

    /// The clearance case reads the resting counts under the bare conversion and
    /// reports what it measured, whatever the verdict on it.
    #[test]
    fn the_clearance_is_measured_off_the_resting_counts() {
        let cfg = undatumed_config();
        let machine = machine_at(&cfg, &rest_legs());
        let (report, _) = run(&cfg, machine);

        // Well under the floor: this configuration is the tight one, and a
        // machine resting there is not one to arm.
        assert_eq!(report.outcome(Case::RestMargins), Outcome::Fail);
        let printed = report.to_string();
        // The clearance the tight configuration leaves, measured back off whole
        // counts. This tree records 0.1411 mm for that configuration; a servo
        // reports whole counts, and rounding the fixture's angles to them moves
        // the clearance by about a micrometre. What a real run reports is this
        // quantised number, never the recorded one.
        assert!(printed.contains("0.1423"), "{printed}");
    }

    /// A limp servo whose goal register does not shadow its present position
    /// fails the case by name, and the reading is on the line either way.
    ///
    /// This is the whole safety argument for enabling torque before writing a
    /// goal. A firmware that stores a limp servo's goal instead of mirroring it
    /// can have that servo enabled against a target somewhere else entirely, and
    /// on this linkage that is the head being slammed. The case has to find it
    /// with the torque still off.
    #[test]
    fn a_limp_servo_whose_goal_is_not_its_present_fails_the_shadow_case() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // A servo that stores its goal, holding one a long way from where it
        // stands: 200 counts is about 17.6°, far outside the 2° gate.
        machine.unmirrored = vec![14];
        let resting = i32::from_le_bytes(
            machine
                .get(14, reg_for(RegId::PresentPosition))
                .expect("the fixture rests somewhere")
                .try_into()
                .expect("a position is four bytes"),
        );
        machine.set(
            14,
            reg_for(RegId::GoalPosition),
            &(resting + 200).to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::GoalShadow), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 14: goal"), "{printed}");
        assert!(
            printed.contains("17.5"),
            "the gap is on the line: {printed}"
        );
        // Evidence, not a reason to stop reading: the cases after it still ran.
        assert_eq!(report.outcome(Case::Datum), Outcome::Pass);
    }

    /// A servo found holding torque is exempt from the assertion and reported
    /// with its gap.
    ///
    /// A torqued servo's goal is a target it really is holding, and the distance
    /// to where it actually sits is the sag of a loaded servo — the number this
    /// project has never measured. Judging it against a tolerance meant for a
    /// mirror would fail every machine that opened the session still holding.
    #[test]
    fn a_torqued_servo_is_exempt_from_the_shadow_assertion_and_still_reported() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        let resting = i32::from_le_bytes(
            machine
                .get(14, reg_for(RegId::PresentPosition))
                .expect("the fixture rests somewhere")
                .try_into()
                .expect("a position is four bytes"),
        );
        machine.set(14, reg_for(RegId::TorqueEnable), &[1]);
        machine.set(
            14,
            reg_for(RegId::GoalPosition),
            &(resting + 200).to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::GoalShadow), Outcome::Pass);
        let printed = report.to_string();
        assert!(printed.contains("14: torque on"), "{printed}");
        assert!(printed.contains("17.5"), "the gap is recorded: {printed}");
        assert!(
            printed.contains("8 limp, 1 holding torque and exempt"),
            "{printed}"
        );
    }

    /// The shadow case reads its own present position rather than reusing the
    /// resting-pose sweep's, and asks every servo whether it is holding torque.
    ///
    /// Asserted off the wire, not off the printed detail: a case that quietly
    /// reused the resting sweep's counts, or filled the torque column in from
    /// nowhere, would print exactly the same lines — and the re-read is what
    /// closes the gap a hand could move the machine through between two sweeps.
    #[test]
    fn the_shadow_case_reads_all_three_registers_from_every_servo() {
        let cfg = undatumed_config();
        let machine = machine_at(&cfg, &stow_legs());
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let (report, _, reads) = run_watched(&cfg, machine);

        assert_eq!(report.outcome(Case::GoalShadow), Outcome::Pass);
        let printed = report.to_string();
        for id in ids {
            let asked = |reg: RegId| {
                reads
                    .iter()
                    .filter(|(servo, addr)| *servo == id && *addr == reg_for(reg).addr)
                    .count()
            };
            // Torque state and goal are read once each, by this case and nothing
            // else in the registry. The present position twice over the whole
            // run: the resting-pose sweep's, and this case's own.
            assert_eq!(asked(RegId::TorqueEnable), 1, "servo {id} torque reads");
            assert_eq!(asked(RegId::GoalPosition), 1, "servo {id} goal reads");
            assert_eq!(
                asked(RegId::PresentPosition),
                2,
                "servo {id} position reads"
            );
            assert!(printed.contains(&format!("{id}: torque off")), "{printed}");
        }
    }

    /// A goal-shadow tolerance the configuration cannot stand behind refuses
    /// before the registry is built.
    ///
    /// An infinite tolerance passes a machine whose goal registers say anything
    /// at all, and writes that pass into the record the arm gate reads. The
    /// registry does not resolve the configuration, so the gate `resolve` gives
    /// every other consumer has to be run here too.
    #[test]
    fn a_shadow_tolerance_that_is_not_an_angle_refuses_the_registry() {
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY] {
            let mut cfg = undatumed_config();
            cfg.arm.goal_shadow_tolerance_deg = bad;
            let Err(refusal) = Registry::from_config(&cfg) else {
                panic!("{bad} is not a tolerance, and a registry was built on it");
            };
            let ConfigError::NotPositive { key, value } = refusal else {
                panic!("{bad}: expected a positivity refusal, got {refusal}");
            };
            assert_eq!(key, "arm.goal_shadow_tolerance_deg");
            assert_eq!(value.to_bits(), bad.to_bits(), "{bad}");
        }
    }

    /// A servo answering a register read with an error number fails the case
    /// that asked, with the number intact.
    #[test]
    fn a_servo_refusing_a_read_fails_the_case_that_asked() {
        let cfg = undatumed_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .errors
            .insert((14, reg_for(RegId::PresentPosition).addr), 7);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::RestPose), Outcome::Fail);
        assert_eq!(report.outcome(Case::Datum), Outcome::NotRun);
        let printed = report.to_string();
        assert!(printed.contains("servo 14 present position"), "{printed}");
    }

    /// The clearance verdict, all three arms, whatever the shipped floor holds.
    ///
    /// [`REST_MARGIN_FLOOR_M`] carries a reviewed number, so a run reaches the
    /// comparison and takes one of the two deciding arms. The third — no floor
    /// at all — is the shape the constant can still be put back into, and it is
    /// asserted here rather than left to execute for the first time on a bench.
    #[test]
    fn the_clearance_verdict_decides_all_three_ways() {
        let floor = 0.000_14;

        let recorded = margin_verdict(0.000_1, None, "as measured");
        assert_eq!(recorded.outcome, Outcome::NotRun);
        assert_eq!(recorded.case, Case::RestMargins);
        assert!(recorded.detail.contains("no reviewed clearance floor"));

        let clear = margin_verdict(0.001, Some(floor), "as measured");
        assert_eq!(clear.outcome, Outcome::Pass);
        assert!(clear.detail.contains("0.1400 mm"), "{}", clear.detail);

        let under = margin_verdict(0.000_1, Some(floor), "as measured");
        assert_eq!(under.outcome, Outcome::Fail);
        assert!(
            under.detail.contains("under the 0.1400 mm floor"),
            "{}",
            under.detail
        );

        // The polarity, stated as the two neighbouring cases rather than as one
        // inequality: swapping the comparison's operands compiles and reads as a
        // ceiling test, and would report a pass at a clearance under the floor.
        assert_eq!(
            margin_verdict(floor * 0.99, Some(floor), "").outcome,
            Outcome::Fail
        );
        assert_eq!(
            margin_verdict(floor * 1.01, Some(floor), "").outcome,
            Outcome::Pass
        );
        // Exactly at the floor is clear of it. Worth pinning before someone
        // tidies the comparison into a strict one.
        assert_eq!(
            margin_verdict(floor, Some(floor), "").outcome,
            Outcome::Pass
        );
        // A clearance nobody can place is not a clearance that cleared.
        assert_eq!(
            margin_verdict(f64::NAN, Some(floor), "").outcome,
            Outcome::Fail
        );
    }

    /// The vendor constant is the alternating quarter turns the legs need and
    /// nothing on the three single-turn joints.
    ///
    /// A quarter turn is 1024 counts, and it is what puts a leg's mechanical
    /// zero at the model's. The parity is what makes the six windows asymmetric
    /// in the two directions they are; getting a sign the wrong way round here
    /// would leave the case passing a machine whose legs read a half turn from
    /// where the model puts them.
    #[test]
    fn the_vendor_offsets_are_alternating_quarter_turns_on_the_legs_alone() {
        let quarter_turn = dxl_proto::conv::COUNTS_PER_REV / 4;
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
            let offset = VENDOR_HOMING_OFFSETS[row];
            match joint {
                JointId::Leg(leg) => {
                    assert_eq!(offset.abs(), quarter_turn, "leg {}", leg + 1);
                    assert_eq!(
                        offset > 0,
                        leg.is_multiple_of(2),
                        "leg {} takes the other sign",
                        leg + 1
                    );
                }
                _ => assert_eq!(offset, 0, "{joint} is a single-turn joint"),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A run in which every case passed.
    fn green() -> SelftestRecord {
        let mut report = Report::new();
        for case in Case::ALL {
            report.push(CaseResult::pass(case, "as expected"));
        }
        report.set_rest_counts([2048; JointId::COUNT]);
        report.set_models([1200, 1200, 1200, 1200, 1200, 1200, 1200, 1020, 1020]);
        report.set_datum(DatumRecord::new(
            DatumSetting::Direct,
            VENDOR_HOMING_OFFSETS,
        ));
        report.into_record(1_754_000_000)
    }

    /// Every case is named exactly once, and the names are distinct and
    /// non-empty — the registry's line format depends on both.
    #[test]
    fn every_case_has_its_own_name() {
        let mut seen = Vec::new();
        for case in Case::ALL {
            let slug = case.slug();
            assert!(!slug.is_empty());
            assert_eq!(slug, case.to_string());
            assert!(!seen.contains(&slug), "{slug} appears twice");
            seen.push(slug);
        }
        assert_eq!(seen.len(), Case::ALL.len());
    }

    /// A case that did not run is a failure, and it is printed rather than
    /// omitted.
    #[test]
    fn a_case_that_did_not_run_is_a_failure_that_still_prints() {
        assert!(!Outcome::NotRun.passed());
        assert!(!Outcome::Fail.passed());
        assert!(Outcome::Pass.passed());

        let mut report = Report::new();
        report.push(CaseResult::pass(Case::PortOpen, "opened at 1000000 baud"));
        report.push(CaseResult::fail(Case::Presence, "servo 14 is silent"));

        assert_eq!(report.outcome(Case::Voltage), Outcome::NotRun);
        assert!(!report.all_passed());

        let printed = report.to_string();
        assert_eq!(printed.lines().count(), Case::ALL.len());
        for case in Case::ALL {
            assert!(printed.contains(case.slug()), "{case} is missing");
        }
        assert!(printed.contains("servo 14 is silent"));
        assert!(printed.contains("did not run"));
    }

    /// A run with every case passing is what a record has to have.
    #[test]
    fn a_run_passes_only_when_every_case_did() {
        let mut report = Report::new();
        for case in Case::ALL {
            assert!(!report.all_passed());
            report.push(CaseResult::pass(case, "ok"));
        }
        assert!(report.all_passed());
        assert_eq!(report.results().len(), Case::ALL.len());
    }

    /// The record round-trips through TOML: what a run wrote is what the next
    /// process reads, cases, readings, offsets and all.
    ///
    /// The signed reading is the half worth pinning. A homing offset is a signed
    /// four-byte register and half of the legs' offsets are negative; rendered as
    /// the unsigned span the register literally holds, a person reviewing the run
    /// reads `4294966272` where the vendor wrote a quarter turn the other way.
    #[test]
    fn a_record_round_trips_through_toml() {
        let record = green();
        let text = record.render().expect("the record renders");
        assert!(text.contains("crank_datum = \"direct\""), "{text}");
        assert!(
            text.contains("-1024"),
            "the negative offsets read as such: {text}"
        );
        assert!(!text.contains("4294966272"), "{text}");

        let read = SelftestRecord::parse(&text).expect("the record parses");
        assert_eq!(read, record);
        let datum = read.datum.expect("the datum row survived");
        assert_eq!(datum.crank_datum, DatumSetting::Direct);
        assert_eq!(datum.homing_offsets, VENDOR_HOMING_OFFSETS);
    }

    /// A datum row naming anything but the one reading this project has is
    /// refused rather than read past.
    ///
    /// There is one datum, established by the servos' own provisioning; a record
    /// claiming another is a file somebody edited, and reading it as `direct`
    /// anyway would put a value nobody wrote in front of the person reviewing the
    /// run.
    #[test]
    fn a_datum_row_naming_another_reading_is_refused() {
        let record = green();
        let text = record.render().expect("the record renders").replace(
            "crank_datum = \"direct\"",
            "crank_datum = \"parity_shifted\"",
        );
        let error = SelftestRecord::parse(&text).expect_err("there is no other reading");
        assert!(error.to_string().contains("parity_shifted"), "{error}");
    }

    /// A record that answers one case twice is refused rather than read past,
    /// and the answer that is read while it is being refused is the worse one.
    ///
    /// The record is a file on disk between the run that wrote it and the arm
    /// that reads it. Two rows for one case say two things about one question,
    /// and a gate that took the first would admit arming on a case that also
    /// says it failed.
    #[test]
    fn a_record_that_answers_a_case_twice_is_refused() {
        let mut record = green();
        record
            .cases
            .push(CaseResult::fail(Case::Voltage, "and then it was not"));

        let text = record.render().expect("the record renders");
        let refused = SelftestRecord::parse(&text).expect_err("two answers is not an answer");
        assert!(refused.to_string().contains("voltage"), "{refused}");

        // And a record built in memory answers with the failure, never the pass
        // that came first.
        assert_eq!(record.outcome(Case::Voltage), Outcome::Fail);
        assert_eq!(
            record.every_case_passed(),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::Voltage,
                outcome: Outcome::Fail,
            })
        );
    }

    /// The verdict names the first case short of a pass and what the record
    /// says about it.
    #[test]
    fn a_case_short_of_a_pass_is_named_by_the_verdict() {
        assert_eq!(green().every_case_passed(), Ok(()));

        let mut record = green();
        record.cases.retain(|result| result.case != Case::Health);
        assert_eq!(
            record.every_case_passed(),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::Health,
                outcome: Outcome::NotRun,
            })
        );

        let mut record = green();
        for result in &mut record.cases {
            if result.case == Case::Voltage {
                result.outcome = Outcome::Fail;
            }
        }
        let refusal = record
            .every_case_passed()
            .expect_err("a failed case is named");
        assert!(refusal.to_string().contains("voltage"), "{refusal}");
    }

    /// A record written before the goal-shadow case existed does not read as a
    /// clean sweep.
    ///
    /// The case establishes the precondition the torque-on path rests on, and a
    /// record that never asked the question has not established it. Nothing
    /// migrates such a file: an absent case reads as not run, which is a
    /// failure, and re-running the self-test is what re-answers the question.
    #[test]
    fn a_record_predating_the_shadow_case_is_not_a_clean_sweep() {
        let mut record = green();
        record
            .cases
            .retain(|result| result.case != Case::GoalShadow);
        let text = record.render().expect("the record renders");
        let read = SelftestRecord::parse(&text).expect("an older record still parses");
        assert_eq!(read.outcome(Case::GoalShadow), Outcome::NotRun);
        assert_eq!(
            read.every_case_passed(),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::GoalShadow,
                outcome: Outcome::NotRun,
            })
        );
    }

    /// The datum is one of the cases, so a machine whose offsets do not
    /// establish it has already failed the sweep rather than needing a check of
    /// its own.
    #[test]
    fn a_failed_datum_case_is_named_by_the_verdict() {
        let mut record = green();
        for result in &mut record.cases {
            if result.case == Case::Datum {
                result.outcome = Outcome::Fail;
            }
        }
        record.datum = None;
        assert_eq!(
            record.every_case_passed(),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::Datum,
                outcome: Outcome::Fail,
            })
        );
    }

    /// A record file with a key nobody wrote is refused rather than read past.
    #[test]
    fn an_unknown_key_in_a_record_is_refused() {
        let text = "taken_at_unix = 1\nwhat_is_this = 2\n";
        assert!(SelftestRecord::parse(text).is_err());

        let text = "taken_at_unix = 1\n\n[datum]\ncrank_datum = \"direct\"\n\
                    homing_offsets = [0, 1024, -1024, 1024, -1024, 1024, -1024, 0, 0]\n\
                    what_is_this = 2\n";
        assert!(SelftestRecord::parse(text).is_err());
    }

    /// The timestamp is a real second, not a placeholder.
    #[test]
    fn the_clock_reads_this_century() {
        // 2026-01-01, comfortably before this code was written.
        assert!(now_unix() > 1_767_225_600);
    }
}
