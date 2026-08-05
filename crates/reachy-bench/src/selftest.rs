//! The read-only self-test registry: what it checks, what a run says, and the
//! record it leaves behind.
//!
//! Every case here reads and nothing here writes: pings, register sweeps, the
//! supply rail, the health bytes, where the platform is resting. No torque, no
//! motion, nothing sent to a servo but a question. That is what makes the
//! registry the gate in front of every command that moves something — it can be
//! run on an unknown machine at no risk, and it is how this project brings
//! hardware up: a case asserts what we expect, and the failure is the discovery.
//!
//! Three rules the shape of this module exists to enforce:
//!
//! - **A case that did not run is a failure, not silence.** [`Outcome::NotRun`]
//!   never counts as a pass, and a case missing from a record reads as
//!   `NotRun` rather than as absent.
//! - **The registry records; a person resolves.** The datum case classifies and
//!   writes down what it saw. Which datum this unit actually has is written
//!   into the bench configuration by a human, with a note saying where it came
//!   from.
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
use reachy_bus::{Bus, BusPort, BusTiming, CrankDatum, RawValue, ServoMap, reg_for, with_retry};
use reachy_kin::{
    EnvelopeConfig, FkOptions, HeadGeometry, below_limit, rest_head_pose, stow_head_pose,
};
use reachy_motion::{
    ArmRecord, JointId, JointVector, ProvisionExpect, ProvisionTable, RegId, RegValue,
};

use crate::config::{BenchConfig, ConfigError, positive};
use crate::datum::{DatumClass, classify_datum, head_height};

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
/// pose's clearance is computed under a datum only after the datum case has
/// classified one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Case {
    /// The serial device opens at the configured rate.
    PortOpen,
    /// Every configured servo answers a ping. Nothing outside the roster is
    /// probed, so this establishes that the nine are there and not that they
    /// are the only ones.
    Presence,
    /// The model numbers fall into the three groups this machine has.
    Identity,
    /// The provisioned setup registers hold what the configuration says.
    ProvisionSweep,
    /// The supply rail is up on all nine servos.
    Voltage,
    /// Nothing is latched in a hardware error status byte.
    Health,
    /// Where the platform is resting, recorded.
    RestPose,
    /// What the resting reading says about the crank datum.
    Datum,
    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations.
    RestMargins,
}

impl Case {
    /// Every case, in run order.
    pub const ALL: [Self; 9] = [
        Self::PortOpen,
        Self::Presence,
        Self::Identity,
        Self::ProvisionSweep,
        Self::Voltage,
        Self::Health,
        Self::RestPose,
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
/// on disk between the run that wrote it and the arm that reads it —
/// [`SelftestRecord::parse`] refuses those outright, and taking the worse
/// answer here means the two gates cannot disagree.
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
/// [`Outcome::NotRun`], so a run that stopped at the presence sweep prints nine
/// lines and fails, rather than printing two and looking short.
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

    /// What the datum case classified.
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

/// What the datum case saw and what it made of it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DatumRecord {
    /// What the reading classified as, and — where it resolved nothing — why.
    #[serde(flatten)]
    pub class: DatumClass,
    /// The six legs as converted counts with no shift applied, degrees — the
    /// reading the classification was made from.
    pub rest_angles_deg: [f64; 6],
    /// The head height the operator measured, metres, if one was measured.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub measured_height_m: Option<f64>,
}

impl DatumRecord {
    /// Record a classification against the reading it was made from.
    #[must_use]
    pub fn new(
        class: DatumClass,
        rest_angles_deg: [f64; 6],
        measured_height_m: Option<f64>,
    ) -> Self {
        Self {
            class,
            rest_angles_deg,
            measured_height_m,
        }
    }
}

/// What a self-test run observed, and when.
///
/// Written beside the bench configuration. It is evidence about a moment, never
/// a cache of machine state: every fact in it is re-established against the
/// hardware by the arm sequence before anything moves.
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
    /// What the datum case classified.
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

    /// A record with no datum row at all.
    #[error(
        "the self-test record carries no datum classification: re-run the self-test before commanding anything"
    )]
    DatumUnclassified,

    /// A record that resolved a datum other than the configured one.
    #[error(
        "the self-test record classified the crank datum as {recorded}, and the configuration says {configured}: one of the two is wrong and no code should pick which"
    )]
    DatumContradiction {
        /// What the record classified.
        recorded: CrankDatum,
        /// What the configuration says.
        configured: CrankDatum,
    },
}

impl SelftestRecord {
    /// What the record says about a case; a case it does not mention did not
    /// run.
    #[must_use]
    pub fn outcome(&self, case: Case) -> Outcome {
        outcome_of(&self.cases, case)
    }

    /// Whether this record admits arming against the configured datum.
    ///
    /// Two conditions, and they are separate: every case passed, and the
    /// record's own classification does not contradict the configuration. A
    /// classification left for human review contradicts nothing — it is the
    /// steady state on a machine resting where membership cannot decide — so a
    /// configured datum with a provenance line stands beside it. A clean
    /// classification disagreeing with configuration is refused outright:
    /// which of the two records is wrong is a person's call.
    ///
    /// TODO(selftest-staleness): a record is admitted here however old it is.
    pub fn admits_arm(&self, configured: CrankDatum) -> Result<(), RecordRefusal> {
        if let Some((case, outcome)) = first_not_passed(&self.cases) {
            return Err(RecordRefusal::CaseNotPassed { case, outcome });
        }

        let datum = self.datum.ok_or(RecordRefusal::DatumUnclassified)?;
        match datum.class.resolved() {
            Some(recorded) if recorded != configured => Err(RecordRefusal::DatumContradiction {
                recorded,
                configured,
            }),
            _ => Ok(()),
        }
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

/// How far the operator's measured head height may sit from where a datum
/// candidate puts the head and still be taken as agreeing, metres.
///
/// A height taken off the bench with a rule is good to a few millimetres. The
/// two candidate readings put the head tens of millimetres apart, so five
/// millimetres accepts an honest measurement while staying far tighter than the
/// distance the test has to resolve.
pub const HEIGHT_TOLERANCE_M: f64 = 0.005;

/// The clearance the resting pose has to leave from the linkage's singular
/// configurations, metres — once a person has read a run and set one.
///
/// `None` until then, and while it is `None` the case reports [`Outcome::NotRun`]
/// and records what it measured. Nobody has established what this platform's
/// rest actually clears, and a threshold invented here would be a number no
/// reading ever had to survive. The consequence is deliberate: a first run
/// cannot admit arming, and filling this in is the review step that turns an
/// observed clearance into a permanent regression guard.
pub const REST_MARGIN_FLOOR_M: Option<f64> = None;

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
/// requires a crank datum and the datum is what this registry exists to
/// classify. Nothing here is a value the registry could send: the only wire
/// traffic it produces is pings and register reads.
#[derive(Clone, Debug)]
pub struct Registry {
    device: String,
    ids: [u8; JointId::COUNT],
    timing: BusTiming,
    /// Counts read with no datum shift applied — the reading the classifier
    /// calls direct, and the only reading available before a datum is resolved.
    direct: ServoMap,
    expected: ProvisionTable,
    env: EnvelopeConfig,
    geom: HeadGeometry,
    fk: FkOptions,
    min_arm_voltage: f64,
    configured_datum: Option<CrankDatum>,
    measured_height_m: Option<f64>,
}

impl Registry {
    /// The registry a configuration describes.
    ///
    /// Only the tables the read-only half needs are converted, so a file with no
    /// `[datum]` table — every file, before the first run — still produces a
    /// runnable registry.
    pub fn from_config(
        cfg: &BenchConfig,
        measured_height_m: Option<f64>,
    ) -> Result<Self, ConfigError> {
        let ids = cfg.servo_ids()?;
        Ok(Self {
            device: cfg.bus.device.clone(),
            ids,
            timing: cfg.bus_timing()?,
            direct: ServoMap::new(ids, CrankDatum::Direct),
            expected: cfg.provision_table(),
            env: cfg.envelope()?,
            geom: HeadGeometry::default(),
            fk: FkOptions::default(),
            // Checked here rather than inherited: resolving the configuration is
            // what normally runs this gate, and the registry deliberately does
            // not resolve — it must run before a datum exists. Without it a
            // non-positive floor would let the voltage case pass a dead rail and
            // write that pass into the record a person reviews.
            min_arm_voltage: positive("arm.min_arm_voltage", cfg.arm.min_arm_voltage)?,
            // Through the gated accessor, never around it: a datum the
            // configuration will not stand behind — absent, or carrying no
            // provenance — reads here as no configured datum, and the clearance
            // case falls back to what this run itself classified.
            configured_datum: cfg.datum().ok().map(|(datum, _)| datum),
            measured_height_m,
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
        let class = self.datum(&counts, report);
        self.rest_margins(&counts, class, report);
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

    /// The model numbers, and whether they fall into the three groups this
    /// machine has: six legs alike, two antennas alike, body yaw its own.
    ///
    /// The values themselves are recorded rather than compared against
    /// expectations: what this unit answers has not been observed yet, and a
    /// number invented here would be one the hardware never had to match.
    fn identity<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let models = match self.sweep_u16(bus, RegId::ModelNumber) {
            Ok(models) => models,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Identity, detail));
                return;
            }
        };
        report.set_models(models);

        let yaw = models[0];
        let legs = &models[1..7];
        let antennas = &models[7..9];
        let detail = format!(
            "body yaw {yaw}, legs {legs:?}, antennas {antennas:?} — recorded for review, no \
             expected value is baked in yet"
        );
        let grouped = legs.iter().all(|model| *model == legs[0])
            && antennas[0] == antennas[1]
            && legs[0] != antennas[0]
            && yaw != legs[0]
            && yaw != antennas[0];
        if grouped {
            report.push(CaseResult::pass(Case::Identity, detail));
        } else {
            report.push(CaseResult::fail(
                Case::Identity,
                format!("the three groups are not three distinct model numbers: {detail}"),
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
    /// Counts rather than angles: a count is what the servo said, and every
    /// reading of it as an angle depends on a datum that is not resolved yet.
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

    /// Classify the datum from the resting reading, and record the
    /// classification.
    ///
    /// The case passes on any classification, review included: classifying is
    /// what it does, and a reading that resolves nothing is a fact about this
    /// machine rather than a failure of the run. Which datum this unit has is
    /// then a person's call, written into the configuration with a note saying
    /// where it came from.
    fn datum(&self, counts: &[i32; JointId::COUNT], report: &mut Report) -> DatumClass {
        let mut rest_direct = [0.0; 6];
        // Walked in bus order and filtered to the legs, so which row a leg's
        // reading arrives on comes from the joint layout rather than from
        // arithmetic here.
        for (row, id) in JointId::ALL.into_iter().enumerate() {
            if let JointId::Leg(leg) = id {
                rest_direct[usize::from(leg)] = counts_to_rad(counts[row]);
            }
        }
        let class = classify_datum(
            &self.env.crank_windows,
            &rest_direct,
            self.measured_height_m,
            |legs| head_height(&self.geom, &self.fk, legs),
            HEIGHT_TOLERANCE_M,
        );
        let rest_deg = rest_direct.map(f64::to_degrees);
        report.set_datum(DatumRecord::new(class, rest_deg, self.measured_height_m));
        let measured = match self.measured_height_m {
            Some(height) => format!("{height:.4} m measured"),
            None => "no height measured".to_string(),
        };
        report.push(CaseResult::pass(
            Case::Datum,
            format!("{class}, from {rest_deg:.3?} deg and {measured}"),
        ));
        class
    }

    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations, under whichever datum is available.
    ///
    /// The configured datum wins over the classification: configuration is what
    /// a person reviewed, and the classification is the evidence they reviewed
    /// it from.
    fn rest_margins(&self, counts: &[i32; JointId::COUNT], class: DatumClass, report: &mut Report) {
        let Some(datum) = self.configured_datum.or_else(|| class.resolved()) else {
            report.push(CaseResult::not_run(
                Case::RestMargins,
                format!(
                    "no datum to read the counts under: the configuration carries none and the \
                     classification is {class}"
                ),
            ));
            return;
        };

        let map = ServoMap::new(self.ids, datum);
        let mut joints = JointVector::default();
        // The joint layout is asked for rather than restated: each reading is
        // filed against the joint that bus row belongs to.
        for (row, (id, count)) in JointId::ALL.into_iter().zip(counts.iter()).enumerate() {
            let angle = match map.present_rad(row, *count) {
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
                    format!("the resting angles close no plausible pose under the {datum} datum: {error}"),
                ));
                return;
            }
        };

        let margins_mm = record.margins.map(|margin| margin * 1000.0);
        let height = record.head_pose_body.translation.z;
        let detail =
            format!("under the {datum} datum: margins {margins_mm:.4?} mm, head at {height:.4} m");
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
        self.direct
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
    use std::cell::RefCell;
    use std::collections::{HashMap, VecDeque};
    use std::io;
    use std::rc::Rc;
    use std::time::Instant;

    use dxl_proto::frame::{HEADER, INST_PING, INST_READ, INST_STATUS};
    use dxl_proto::{Reg, crc16, rad_to_counts};
    use reachy_kin::{LegAngles, inverse_kinematics};

    use super::*;
    use crate::datum::ReviewReason;

    /// Model numbers the three groups answer with in these fixtures. Not this
    /// unit's values — nobody has read them yet, and the case records rather
    /// than compares.
    const YAW_MODEL: u16 = 1190;
    const LEG_MODEL: u16 = 1200;
    const ANTENNA_MODEL: u16 = 1020;

    /// A rail comfortably over the arm floor, in the register's tenths of a volt.
    const HEALTHY_RAIL: u16 = 118;

    /// A scripted machine: nine servos with a register file, answering pings and
    /// reads over the port seam.
    ///
    /// It answers nothing else: an instruction it does not implement gets
    /// silence, and the [`Spy`] wrapped around it is what a test reads the
    /// traffic back from.
    struct FakeMachine {
        regs: HashMap<(u8, u16), Vec<u8>>,
        errors: HashMap<(u8, u16), u8>,
        silent: Vec<u8>,
        out: VecDeque<u8>,
    }

    impl FakeMachine {
        fn new() -> Self {
            Self {
                regs: HashMap::new(),
                errors: HashMap::new(),
                silent: Vec::new(),
                out: VecDeque::new(),
            }
        }

        fn set(&mut self, id: u8, reg: Reg, bytes: &[u8]) {
            self.regs.insert((id, reg.addr), bytes.to_vec());
        }

        /// A status frame as a servo puts it on the wire.
        fn reply(&mut self, id: u8, error: u8, params: &[u8]) {
            let mut frame = Vec::from(HEADER);
            frame.push(id);
            let len = u16::try_from(params.len() + 4).expect("a fixture reply is short");
            frame.extend_from_slice(&len.to_le_bytes());
            frame.push(INST_STATUS);
            frame.push(error);
            frame.extend_from_slice(params);
            frame.extend_from_slice(&crc16(&frame).to_le_bytes());
            self.out.extend(frame);
        }
    }

    impl BusPort for FakeMachine {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            let id = buf[4];
            let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
            let instruction = buf[7];
            let params = &buf[8..8 + len - 3];
            if self.silent.contains(&id) {
                return Ok(());
            }
            match instruction {
                INST_PING => {
                    let model = self
                        .regs
                        .get(&(id, 0))
                        .cloned()
                        .unwrap_or_else(|| vec![0, 0]);
                    self.reply(id, 0, &[model[0], model[1], 42]);
                }
                INST_READ => {
                    let addr = u16::from_le_bytes([params[0], params[1]]);
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    let error = self.errors.get(&(id, addr)).copied().unwrap_or(0);
                    let mut value = self.regs.get(&(id, addr)).cloned().unwrap_or_default();
                    value.resize(width, 0);
                    self.reply(id, error, &value);
                }
                // Anything else is a fixture that was asked to do something this
                // machine does not do; silence makes the caller time out and the
                // recorded instruction makes the test say why.
                _ => {}
            }
            Ok(())
        }

        fn read_some(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            let mut taken = 0;
            while taken < buf.len() {
                match self.out.pop_front() {
                    Some(byte) => {
                        buf[taken] = byte;
                        taken += 1;
                    }
                    None => break,
                }
            }
            Ok(taken)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            self.out.clear();
            Ok(())
        }
    }

    /// The configuration the example ships with, which is what an operator
    /// copies. It carries no datum table — the first run has no way to have one
    /// — so it is exactly the file a bring-up starts from.
    fn undatumed_config() -> BenchConfig {
        let cfg =
            crate::config::parse(include_str!("../reachy-bench.example.toml")).expect("it parses");
        assert_eq!(cfg.datum, None, "the shipped example resolves no datum");
        cfg
    }

    /// The same file after a person has reviewed a run and written the datum in,
    /// with whatever they wrote as its provenance.
    fn datumed_config(provenance: &str) -> BenchConfig {
        crate::config::parse(&format!(
            "{}\n[datum]\ncrank_datum = \"direct\"\nprovenance = \"{provenance}\"\n",
            include_str!("../reachy-bench.example.toml")
        ))
        .expect("it parses")
    }

    /// A machine holding exactly what the configuration says it should, resting
    /// at `legs`.
    fn machine_at(cfg: &BenchConfig, legs: &[f64; 6]) -> FakeMachine {
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let map = ServoMap::new(ids, CrankDatum::Direct);
        let table = cfg.provision_table();
        let mut machine = FakeMachine::new();

        for (row, id) in ids.iter().enumerate() {
            let model = match row {
                0 => YAW_MODEL,
                1..=6 => LEG_MODEL,
                _ => ANTENNA_MODEL,
            };
            machine.set(*id, reg_for(RegId::ModelNumber), &model.to_le_bytes());
            machine.set(
                *id,
                reg_for(RegId::PresentInputVoltage),
                &HEALTHY_RAIL.to_le_bytes(),
            );
            machine.set(*id, reg_for(RegId::HardwareErrorStatus), &[0]);
            let angle = match row {
                0 => 0.0,
                1..=6 => legs[row - 1],
                _ => 0.0,
            };
            let counts = rad_to_counts(angle).expect("a resting angle places");
            machine.set(*id, reg_for(RegId::PresentPosition), &counts.to_le_bytes());
        }

        for reg in RegId::ALL {
            let Some(column) = ProvisionTable::column(reg) else {
                continue;
            };
            for (row, id) in ids.iter().enumerate() {
                let entry = reg_for(reg);
                match table.at(row, column) {
                    Some(ProvisionExpect::Check(value)) => {
                        let raw = map
                            .encode_value(row, reg, value)
                            .expect("a configured expectation encodes");
                        machine.set(*id, entry, raw.as_slice());
                    }
                    // A recorded register holds whatever it holds; zero is a
                    // value like any other and the case does not judge it.
                    Some(ProvisionExpect::Record) => {
                        machine.set(*id, entry, &vec![0u8; usize::from(entry.len)]);
                    }
                    _ => {}
                }
            }
        }
        machine
    }

    /// The six crank angles the stow pose holds.
    fn stow_legs() -> [f64; 6] {
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&HeadGeometry::default(), &stow_head_pose(), &mut angles)
            .expect("the stow pose is reachable");
        angles.0
    }

    /// The six crank angles the tight resting configuration holds.
    fn rest_legs() -> [f64; 6] {
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&HeadGeometry::default(), &rest_head_pose(), &mut angles)
            .expect("the resting pose is reachable");
        angles.0
    }

    /// The head height a set of crank angles implies — the probe the classifier
    /// is handed, and the registry's own.
    fn probe_height(legs: &[f64; 6]) -> Option<f64> {
        head_height(&HeadGeometry::default(), &FkOptions::default(), legs)
    }

    /// A run of the registry against a machine, with the port already open, and
    /// every instruction that crossed the wire.
    fn run(
        cfg: &BenchConfig,
        machine: FakeMachine,
        height: Option<f64>,
    ) -> (Report, Vec<(u8, u8)>) {
        let registry = Registry::from_config(cfg, height).expect("the configuration converts");
        let log = Rc::new(RefCell::new(Vec::new()));
        let spy = Spy {
            inner: machine,
            log: Rc::clone(&log),
        };
        let mut report = Report::new();
        registry.run(Ok::<Spy, String>(spy), &mut report);
        let instructions = log.borrow().clone();
        (report, instructions)
    }

    /// A port that records every instruction that crosses it.
    struct Spy {
        inner: FakeMachine,
        log: Rc<RefCell<Vec<(u8, u8)>>>,
    }

    impl BusPort for Spy {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            self.log.borrow_mut().push((buf[4], buf[7]));
            self.inner.write_all(buf)
        }

        fn read_some(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize> {
            self.inner.read_some(buf, deadline)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            self.inner.discard_input()
        }
    }

    /// A machine holding what it should, resting at the stow pose with the
    /// height that pose implies, passes every case the registry can decide —
    /// and the clearance case still reports `NotRun`, because no reviewed floor
    /// exists to assert against.
    #[test]
    fn a_correct_machine_passes_every_case_but_the_unreviewed_clearance() {
        let cfg = undatumed_config();
        let pose = stow_head_pose();
        let machine = machine_at(&cfg, &stow_legs());
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

        for case in Case::ALL {
            let expected = if case == Case::RestMargins {
                Outcome::NotRun
            } else {
                Outcome::Pass
            };
            assert_eq!(
                report.outcome(case),
                expected,
                "{case}: {}",
                report
                    .results()
                    .iter()
                    .find(|result| result.case == case)
                    .map_or_else(|| "no verdict".to_string(), ToString::to_string)
            );
        }
        assert!(!report.all_passed(), "an unreviewed floor is not a pass");

        let record = report.into_record(1);
        assert!(record.rest_counts.is_some());
        assert_eq!(
            record.models,
            Some([
                YAW_MODEL,
                LEG_MODEL,
                LEG_MODEL,
                LEG_MODEL,
                LEG_MODEL,
                LEG_MODEL,
                LEG_MODEL,
                ANTENNA_MODEL,
                ANTENNA_MODEL
            ])
        );
        let datum = record.datum.expect("the datum case recorded");
        assert_eq!(datum.class, DatumClass::Direct);
        assert_eq!(datum.measured_height_m, Some(pose.translation.z));
    }

    /// The registry pings and reads, addresses nothing but its own roster, and
    /// does nothing else. This is the property that makes it safe to run on an
    /// unknown machine — and the presence case prints it as a statement of
    /// fact, so both halves are asserted here rather than only the instruction.
    #[test]
    fn the_registry_only_ever_pings_and_reads_its_own_roster() {
        let cfg = undatumed_config();
        let pose = stow_head_pose();
        let machine = machine_at(&cfg, &stow_legs());
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let (_, instructions) = run(&cfg, machine, Some(pose.translation.z));

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
            let refusal =
                Registry::from_config(&cfg, None).expect_err("{bad} is not a supply floor");
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
        let registry = Registry::from_config(&cfg, None).expect("the configuration converts");
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
        let pose = stow_head_pose();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.silent = vec![12, 17];
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

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
        let pose = stow_head_pose();
        let mut machine = machine_at(&cfg, &stow_legs());
        // Operating mode 0 on one servo: the reading that voids the servo-side
        // position envelope.
        machine.set(13, reg_for(RegId::OperatingMode), &[0]);
        machine.set(16, reg_for(RegId::TemperatureLimit), &[95]);
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 13 operating mode"), "{printed}");
        assert!(printed.contains("servo 16 temperature limit"), "{printed}");
        // The cases after it still ran: a provisioning disagreement is evidence,
        // not a reason to stop reading.
        assert_eq!(report.outcome(Case::Voltage), Outcome::Pass);
        assert_eq!(report.outcome(Case::Health), Outcome::Pass);
    }

    /// A rail under the arm floor fails, and the reading is reported either way.
    #[test]
    fn a_rail_under_the_arm_floor_fails_and_says_which_servo() {
        let cfg = undatumed_config();
        let pose = stow_head_pose();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            15,
            reg_for(RegId::PresentInputVoltage),
            &45u16.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

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
        let pose = stow_head_pose();
        let legs = stow_legs();

        let mut voltage_only = machine_at(&cfg, &legs);
        voltage_only.set(11, reg_for(RegId::HardwareErrorStatus), &[0x01]);
        let (report, _) = run(&cfg, voltage_only, Some(pose.translation.z));
        assert_eq!(report.outcome(Case::Health), Outcome::Pass);
        assert!(report.to_string().contains("0x01"), "the byte is reported");

        let mut overload = machine_at(&cfg, &legs);
        overload.set(11, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let (report, _) = run(&cfg, overload, Some(pose.translation.z));
        assert_eq!(report.outcome(Case::Health), Outcome::Fail);
        assert!(
            report.to_string().contains("11 = 0x20"),
            "the servo is named"
        );
    }

    /// The model numbers have to be three distinct groups; two groups answering
    /// alike is a machine that is not wired the way this project thinks.
    #[test]
    fn the_model_numbers_must_be_three_distinct_groups() {
        let cfg = undatumed_config();
        let pose = stow_head_pose();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(17, reg_for(RegId::ModelNumber), &LEG_MODEL.to_le_bytes());
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

        assert_eq!(report.outcome(Case::Identity), Outcome::Fail);
        assert!(
            report
                .to_string()
                .contains("not three distinct model numbers"),
            "{report}"
        );
        // Recorded regardless: the reading is the evidence a person reviews.
        let record = report.into_record(1);
        assert_eq!(
            record.models.map(|models| models[7]),
            Some(LEG_MODEL),
            "the observed value is recorded even when it fails"
        );
    }

    /// A machine resting at the tight configuration classifies for review, which
    /// is the steady state there — and with no datum in configuration and none
    /// classified, the clearance case has nothing to read the counts under.
    #[test]
    fn the_tight_resting_configuration_classifies_for_review_and_leaves_no_datum() {
        let cfg = undatumed_config();
        let legs = rest_legs();
        let machine = machine_at(&cfg, &legs);
        let (report, _) = run(&cfg, machine, Some(rest_head_pose().translation.z));

        assert_eq!(report.outcome(Case::Datum), Outcome::Pass);
        assert_eq!(report.outcome(Case::RestMargins), Outcome::NotRun);
        let printed = report.to_string();
        assert!(printed.contains("for human review"), "{printed}");
        assert!(
            printed.contains("no datum to read the counts under"),
            "{printed}"
        );

        let record = report.into_record(1);
        let datum = record.datum.expect("a classification was recorded");
        assert_eq!(
            datum.class,
            DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB)
        );
    }

    /// With a datum in configuration the clearance case reads the counts under
    /// it and reports what it measured, floor or no floor.
    #[test]
    fn a_configured_datum_is_what_the_clearance_is_measured_under() {
        let cfg = datumed_config("test fixture");
        assert!(cfg.datum().is_ok(), "the gate stands behind it");
        let legs = rest_legs();
        let machine = machine_at(&cfg, &legs);
        let (report, _) = run(&cfg, machine, Some(rest_head_pose().translation.z));

        assert_eq!(report.outcome(Case::RestMargins), Outcome::NotRun);
        let printed = report.to_string();
        assert!(printed.contains("under the direct datum"), "{printed}");
        assert!(printed.contains("no reviewed clearance floor"), "{printed}");
        // The clearance the tight configuration leaves, measured back off whole
        // counts. This tree records 0.1411 mm for that configuration; a servo
        // reports whole counts, and rounding the fixture's angles to them moves
        // the clearance by about a micrometre. What a real run reports is this
        // quantised number, never the recorded one.
        assert!(printed.contains("0.1423"), "{printed}");
    }

    /// A `[datum]` table the configuration will not stand behind is not a
    /// configured datum here either. The registry asks through the same gate
    /// every motion command asks through, so a clearance is never measured
    /// under a datum that cannot resolve.
    #[test]
    fn an_unprovenanced_datum_is_no_configured_datum() {
        let cfg = datumed_config("   ");
        assert!(cfg.datum.is_some(), "the table is there");
        assert!(cfg.datum().is_err(), "the gate refuses it");

        let legs = rest_legs();
        let machine = machine_at(&cfg, &legs);
        let (report, _) = run(&cfg, machine, Some(rest_head_pose().translation.z));

        // The tight rest classifies for review, so with the configured datum
        // out of play there is nothing left to read the counts under.
        assert_eq!(report.outcome(Case::RestMargins), Outcome::NotRun);
        let printed = report.to_string();
        assert!(
            printed.contains("no datum to read the counts under"),
            "{printed}"
        );
        assert!(!printed.contains("under the direct datum"), "{printed}");
    }

    /// A servo answering a register read with an error number fails the case
    /// that asked, with the number intact.
    #[test]
    fn a_servo_refusing_a_read_fails_the_case_that_asked() {
        let cfg = undatumed_config();
        let pose = stow_head_pose();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .errors
            .insert((14, reg_for(RegId::PresentPosition).addr), 7);
        let (report, _) = run(&cfg, machine, Some(pose.translation.z));

        assert_eq!(report.outcome(Case::RestPose), Outcome::Fail);
        assert_eq!(report.outcome(Case::Datum), Outcome::NotRun);
        let printed = report.to_string();
        assert!(printed.contains("servo 14 present position"), "{printed}");
    }

    /// The clearance verdict, all three arms, whatever the shipped floor holds.
    ///
    /// [`REST_MARGIN_FLOOR_M`] is `None` today, so nothing on a run reaches the
    /// comparison. It is filled in from the first reviewed hardware run, and the
    /// arm that then decides whether a resting pose is far enough from the
    /// linkage's singular configurations must not be executing for the first
    /// time when it does.
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

    /// The height tolerance is tight against the thing it has to resolve.
    ///
    /// The height test is the only one in the classifier carrying information
    /// from outside the model, and it is the one that decides between the two
    /// readings. Its tolerance is therefore bounded from above by the distance
    /// between the two candidates' heads: widen it past that gap and a machine
    /// on the wrong datum classifies clean, a person writes that datum into the
    /// configuration, and the head is commanded from tens of millimetres away
    /// from where the model thinks it is.
    #[test]
    fn the_height_tolerance_is_far_inside_the_gap_it_resolves() {
        let legs = stow_legs();
        let under_model = probe_height(&legs).expect("the model's own angles close");
        // The same machine read under the other datum: what the direct reading
        // would be if the truth were the shifted one.
        let under_other = probe_height(&crate::datum::shifted_reading(&legs))
            .expect("the shifted candidate closes too");
        let gap = (under_model - under_other).abs();
        assert!(
            gap > 0.02,
            "the two candidates are {gap} m apart, which is not a gap a rule resolves"
        );
        assert!(
            HEIGHT_TOLERANCE_M < gap / 4.0,
            "a {HEIGHT_TOLERANCE_M} m tolerance is not far inside a {gap} m gap"
        );
    }

    /// The boundary the tolerance draws, from both sides.
    ///
    /// Without this the constant could be widened several-fold with the suite
    /// green, which is the one change it exists to prevent.
    #[test]
    fn the_height_tolerance_decides_at_its_own_boundary() {
        let legs = stow_legs();
        let height = probe_height(&legs).expect("the stow configuration closes");
        let windows = EnvelopeConfig::default().crank_windows;
        let classify = |measured: f64| {
            crate::datum::classify_datum(
                &windows,
                &legs,
                Some(measured),
                probe_height,
                HEIGHT_TOLERANCE_M,
            )
        };

        for side in [-1.0, 1.0] {
            assert_eq!(
                classify(height + side * HEIGHT_TOLERANCE_M * 0.99),
                DatumClass::Direct,
                "a measurement just inside the tolerance agrees"
            );
            assert_eq!(
                classify(height + side * HEIGHT_TOLERANCE_M * 1.01),
                DatumClass::HumanReview(ReviewReason::HeightMismatch),
                "a measurement just outside it does not"
            );
        }
    }

    /// What the height tolerance is bounded by is the rule, not the model.
    ///
    /// The two records this tree holds of the resting configuration — six crank
    /// angles, and the head pose recovered from them — disagree by micrometres
    /// when both are put through the solver. The tolerance is orders of
    /// magnitude looser than that, so what it is really sized against is how
    /// well a person can measure a head height, which is the intended reading.
    #[test]
    fn the_height_tolerance_is_bounded_by_the_rule_and_not_by_the_model() {
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let from_angles = head_height(&geom, &fk, &rest_legs()).expect("the resting angles close");
        let from_pose = rest_head_pose().translation.z;
        let model_disagreement = (from_angles - from_pose).abs();
        assert!(
            model_disagreement < HEIGHT_TOLERANCE_M / 100.0,
            "the model disagrees with itself by {model_disagreement} m, against a \
             {HEIGHT_TOLERANCE_M} m tolerance"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datum::ReviewReason;

    /// A run in which every case passed, with a clean classification.
    fn green(class: DatumClass) -> SelftestRecord {
        let mut report = Report::new();
        for case in Case::ALL {
            report.push(CaseResult::pass(case, "as expected"));
        }
        report.set_rest_counts([2048; JointId::COUNT]);
        report.set_models([1200, 1200, 1200, 1200, 1200, 1200, 1200, 1020, 1020]);
        report.set_datum(DatumRecord::new(class, [10.0; 6], Some(0.1266)));
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
    /// process reads, cases, readings, classification and all.
    #[test]
    fn a_record_round_trips_through_toml() {
        let record = green(DatumClass::Direct);
        let text = record.render().expect("the record renders");
        let read = SelftestRecord::parse(&text).expect("the record parses");
        assert_eq!(read, record);

        // And the review arm, whose reason is a second field.
        let reviewed = green(DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB));
        let text = reviewed.render().expect("the record renders");
        assert!(text.contains("shifted-at-candidate-b"), "{text}");
        assert_eq!(
            SelftestRecord::parse(&text).expect("the record parses"),
            reviewed
        );
    }

    /// Every classification survives a round trip through the record file, and
    /// arrives on the arming gate as itself.
    ///
    /// The serde contract is not free: the class is adjacently tagged, renamed
    /// to kebab-case and flattened into a struct that refuses unknown keys. A
    /// rename typo on a variant nothing round-trips would write a record the
    /// parser cannot read, and a variant that came back as the wrong arm would
    /// turn a `parity-shifted` reading into `direct` on the gate in front of
    /// every motion command. The wire name is asserted for each, so a rename is
    /// a visible diff rather than a silent one.
    #[test]
    fn every_classification_survives_the_record() {
        let classes = [
            (DatumClass::Direct, "direct"),
            (DatumClass::ParityShifted, "parity-shifted"),
            (DatumClass::HumanReview(ReviewReason::Neither), "neither"),
            (DatumClass::HumanReview(ReviewReason::Both), "both"),
            (
                DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB),
                "shifted-at-candidate-b",
            ),
            (
                DatumClass::HumanReview(ReviewReason::FkInconsistent),
                "fk-inconsistent",
            ),
            (
                DatumClass::HumanReview(ReviewReason::HeightUnmeasured),
                "height-unmeasured",
            ),
            (
                DatumClass::HumanReview(ReviewReason::HeightMismatch),
                "height-mismatch",
            ),
        ];
        for (class, wire_name) in classes {
            let record = green(class);
            let text = record.render().expect("the record renders");
            assert!(text.contains(wire_name), "{class:?} renders as: {text}");
            let read = SelftestRecord::parse(&text).expect("the record parses");
            assert_eq!(read, record, "{class:?}");
            let datum = read.datum.expect("the datum row survived");
            assert_eq!(datum.class, class);
            assert_eq!(datum.class.resolved(), class.resolved());
        }
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
        let mut record = green(DatumClass::Direct);
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
            record.admits_arm(CrankDatum::Direct),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::Voltage,
                outcome: Outcome::Fail,
            })
        );
    }

    /// Every case must pass before anything is commanded, and the refusal names
    /// the case and what the record says about it.
    #[test]
    fn a_case_short_of_a_pass_refuses_arming() {
        let mut record = green(DatumClass::Direct);
        record.cases.retain(|result| result.case != Case::Health);
        assert_eq!(
            record.admits_arm(CrankDatum::Direct),
            Err(RecordRefusal::CaseNotPassed {
                case: Case::Health,
                outcome: Outcome::NotRun,
            })
        );

        let mut record = green(DatumClass::Direct);
        for result in &mut record.cases {
            if result.case == Case::Voltage {
                result.outcome = Outcome::Fail;
            }
        }
        let refusal = record
            .admits_arm(CrankDatum::Direct)
            .expect_err("a failed case refuses");
        assert!(refusal.to_string().contains("voltage"), "{refusal}");
    }

    /// A clean classification that contradicts the configuration refuses, and
    /// says both records.
    #[test]
    fn a_clean_classification_against_the_configured_datum_refuses() {
        let record = green(DatumClass::Direct);
        assert_eq!(record.admits_arm(CrankDatum::Direct), Ok(()));

        let refusal = record
            .admits_arm(CrankDatum::ParityShifted)
            .expect_err("the two records disagree");
        assert_eq!(
            refusal,
            RecordRefusal::DatumContradiction {
                recorded: CrankDatum::Direct,
                configured: CrankDatum::ParityShifted,
            }
        );
        let printed = refusal.to_string();
        assert!(
            printed.contains("direct") && printed.contains("parity shifted"),
            "{printed}"
        );
    }

    /// The agreement rule's positive case: a record left for human review arms
    /// under either configured datum, because it contradicts neither. It is the
    /// steady state on a machine resting where membership cannot decide.
    #[test]
    fn a_reviewed_classification_arms_under_either_datum() {
        let record = green(DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB));
        assert_eq!(record.admits_arm(CrankDatum::Direct), Ok(()));
        assert_eq!(record.admits_arm(CrankDatum::ParityShifted), Ok(()));
    }

    /// A green record with no datum row at all refuses rather than arming on a
    /// classification nobody made.
    #[test]
    fn a_record_with_no_classification_refuses() {
        let mut record = green(DatumClass::Direct);
        record.datum = None;
        assert_eq!(
            record.admits_arm(CrankDatum::Direct),
            Err(RecordRefusal::DatumUnclassified)
        );
    }

    /// A record file with a key nobody wrote is refused rather than read past.
    #[test]
    fn an_unknown_key_in_a_record_is_refused() {
        let text = "taken_at_unix = 1\nwhat_is_this = 2\n";
        assert!(SelftestRecord::parse(text).is_err());

        let text = "taken_at_unix = 1\n\n[datum]\nclassification = \"direct\"\n\
                    rest_angles_deg = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]\nwhat_is_this = 2\n";
        assert!(SelftestRecord::parse(text).is_err());
    }

    /// A `human-review` row whose reason is missing is refused, not read as
    /// some particular reason. A record that lost half of what it says is
    /// evidence of nothing, and substituting a plausible reason would put a
    /// reading nobody took in front of the person reviewing the run.
    #[test]
    fn a_review_row_with_no_reason_is_refused() {
        let text = "taken_at_unix = 1\n\n[datum]\nclassification = \"human-review\"\n\
                    rest_angles_deg = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]\n";
        let error = SelftestRecord::parse(text).expect_err("a reason-less review row is refused");
        assert!(error.to_string().contains("review_reason"), "{error}");

        let text = format!("{text}review_reason = \"both\"\n");
        let record = SelftestRecord::parse(&text).expect("a complete review row parses");
        let datum = record.datum.expect("the datum row is there");
        assert_eq!(datum.class, DatumClass::HumanReview(ReviewReason::Both));
    }

    /// The timestamp is a real second, not a placeholder.
    #[test]
    fn the_clock_reads_this_century() {
        // 2026-01-01, comfortably before this code was written.
        assert!(now_unix() > 1_767_225_600);
    }
}
