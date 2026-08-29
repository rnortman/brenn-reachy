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
//! - **The registry records what it measured.** The datum case reads the
//!   servos' provisioned homing offsets, compares them against the vendor
//!   offsets baked into the motion layer, and writes down what it saw. That
//!   record is the datum: nothing else in the tree carries one.
//! - **The record is evidence, not memory.** It says what was observed and
//!   when. Nothing reads it to find out what state the machine is in now —
//!   arming re-verifies every one of these facts against the hardware on every
//!   run.

use std::fmt;
use std::path::Path;
use std::time::Duration;

use anyhow::Context as _;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use dxl_proto::conv::COUNTS_PER_REV;
use dxl_proto::{HardwareError, counts_to_rad, volts_from_raw};
use reachy_bus::{
    Bus, BusPort, BusTiming, CYCLE_HOST_ALLOWANCE, ExchangeSpans, RawValue, ServoMap,
    SyncReadOutcome, named_reg, with_retry,
};
use reachy_kin::{
    EnvelopeConfig, FkOptions, HeadGeometry, below_limit, outside_limit, rest_head_pose,
    stow_head_pose,
};
use reachy_motion::arm::{WINDOW_INSET_DEG, leg_windows};
use reachy_motion::joints::{LEG_COUNT, ROW_COUNT, ROWS, group_of, leg_index};
use reachy_motion::reg::{self, Name as RegName};
use reachy_motion::{
    ArmRecord, EXPECTED_MODELS, JointGroup, JointVector, ProvisionExpect, ProvisionTable, RegId,
    Shown, VENDOR_HOMING_OFFSETS, Value, value,
};

use crate::bare;
use crate::config::{BenchConfig, ConfigError, positive};

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
    /// Each leg servo's own travel fence agrees with the motion envelope the
    /// commanding host runs.
    LegFence,
    /// The supply rail is up on all nine servos.
    Voltage,
    /// Every servo's temperature reads inside the band a resting machine sits
    /// in.
    Temperature,
    /// Nothing is latched in a hardware error status byte.
    Health,
    /// An exchange costs what a control cycle budgets for it.
    BusExchangeTiming,
    /// Where the platform is resting, recorded.
    RestPose,
    /// Every limp servo reports its goal as its present position, which is what
    /// makes enabling torque before writing a goal safe.
    GoalShadow,
    /// The provisioned homing offsets are the vendor's, so a converted count is
    /// the model's crank angle.
    Datum,
    /// Each antenna is resting inside the one turn a boot fold leaves it in.
    AntennaFold,
    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations.
    RestMargins,
}

impl Case {
    /// Every case, in run order.
    pub const ALL: [Self; 14] = [
        Self::PortOpen,
        Self::Presence,
        Self::Identity,
        Self::ProvisionSweep,
        Self::LegFence,
        Self::Voltage,
        Self::Temperature,
        Self::Health,
        Self::BusExchangeTiming,
        Self::RestPose,
        Self::GoalShadow,
        Self::Datum,
        Self::AntennaFold,
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
            Self::LegFence => "leg-fence",
            Self::Voltage => "voltage",
            Self::Temperature => "temperature",
            Self::Health => "health",
            Self::BusExchangeTiming => "bus-exchange-timing",
            Self::RestPose => "rest-pose",
            Self::GoalShadow => "goal-shadow",
            Self::Datum => "datum",
            Self::AntennaFold => "antenna-fold",
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
    rest_counts: Option<[i32; ROW_COUNT]>,
    models: Option<[u16; ROW_COUNT]>,
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
    pub fn set_rest_counts(&mut self, counts: [i32; ROW_COUNT]) {
        self.rest_counts = Some(counts);
    }

    /// The model numbers, in bus order.
    pub fn set_models(&mut self, models: [u16; ROW_COUNT]) {
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

/// The datum as the self-test record spells it.
///
/// One variant, deliberately. A host-side correction is never the answer to a
/// servo that lacks its provisioned offset: that is one servo's fault, refused
/// by name, and a compensating shift would move all six legs for it. The enum
/// exists so that a record saying anything else is refused by serde rather than
/// read past.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumSetting {
    /// A converted count is the crank angle as the model means it, because each
    /// leg servo applies its provisioned homing offset before reporting.
    Direct,
}

impl fmt::Display for DatumSetting {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Direct => f.write_str("direct"),
        }
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
    pub homing_offsets: [i32; ROW_COUNT],
}

impl DatumRecord {
    /// Record the datum the observed offsets establish.
    #[must_use]
    pub fn new(crank_datum: DatumSetting, homing_offsets: [i32; ROW_COUNT]) -> Self {
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
    pub rest_counts: Option<[i32; ROW_COUNT]>,
    /// The model numbers, bus order — recorded for human review before any
    /// expected value is baked into the identity case.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub models: Option<[u16; ROW_COUNT]>,
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
    /// passes only when all nine homing offsets are the vendor's — the evidence
    /// every converted angle rests on, recorded here and nowhere else.
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

/// Exchanges of each kind [`Case::BusExchangeTiming`] measures.
///
/// Two hundred is enough for a ninety-ninth percentile to mean something — at
/// this count the nearest rank is the 198th of the sorted two hundred, the
/// third-worst — and small enough that the case costs a few seconds of a
/// read-only run.
const TIMING_EXCHANGES: usize = 200;

/// A measured distribution, at the four places a person reads one at.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct SpanStats {
    min: Duration,
    median: Duration,
    p99: Duration,
    max: Duration,
}

impl SpanStats {
    /// The four places of `spans`, which arrive in whatever order they were
    /// measured.
    ///
    /// Nearest-rank percentiles over the sorted spans: no interpolation, so
    /// every figure printed is a span some exchange actually took. An empty set
    /// reads as zeroes, which is what a case that measured nothing has to say.
    fn of(spans: &[Duration]) -> Self {
        let mut sorted: Vec<Duration> = spans.to_vec();
        sorted.sort_unstable();
        let at = |quantile: f64| -> Duration {
            if sorted.is_empty() {
                return Duration::ZERO;
            }
            // The rank of a quantile over n spans, as an index: at least the
            // first and never past the last. Not a bound on anything commanded
            // — it is where in a sorted list to look.
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "a rank over at most a few hundred spans is a small positive integer"
            )]
            let rank = (quantile * sorted.len() as f64).ceil() as usize;
            sorted[rank.max(1).min(sorted.len()) - 1]
        };
        Self {
            min: at(0.0),
            median: at(0.5),
            p99: at(0.99),
            max: at(1.0),
        }
    }
}

impl fmt::Display for SpanStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}/{}",
            millis(self.min),
            millis(self.median),
            millis(self.p99),
            millis(self.max)
        )
    }
}

/// A span in milliseconds, as every span in this module prints.
fn millis(span: Duration) -> String {
    format!("{:.3}", span.as_secs_f64() * 1e3)
}

/// One kind of exchange, measured against the budget it has to fit.
#[derive(Clone, Debug)]
struct TimingRun {
    /// What was exchanged, for the line the case prints.
    what: &'static str,
    /// What one of these may cost inside a control cycle.
    bound: Duration,
    spans: Vec<ExchangeSpans>,
}

impl TimingRun {
    fn new(what: &'static str, bound: Duration) -> Self {
        Self {
            what,
            bound,
            spans: Vec::with_capacity(TIMING_EXCHANGES),
        }
    }

    fn note(&mut self, spans: ExchangeSpans) {
        self.spans.push(spans);
    }

    fn len(&self) -> usize {
        self.spans.len()
    }

    /// Exchanges whose total ran past the budget. The assertion the case makes,
    /// counted rather than stopped at the first: how many of them overran is
    /// the difference between a stall and a bus that is simply this slow.
    fn overruns(&self) -> usize {
        self.spans
            .iter()
            .filter(|spans| spans.total() > self.bound)
            .count()
    }

    /// The distribution of one of the three spans.
    fn stats(&self, pick: impl Fn(&ExchangeSpans) -> Duration) -> SpanStats {
        let spans: Vec<Duration> = self.spans.iter().map(pick).collect();
        SpanStats::of(&spans)
    }
}

impl fmt::Display for TimingRun {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} x{} against {} ms, min/median/p99/max ms: total {}, write {}, wait {}",
            self.what,
            self.len(),
            millis(self.bound),
            self.stats(ExchangeSpans::total),
            self.stats(|spans| spans.send),
            self.stats(|spans| spans.wait)
        )
    }
}

/// What the two measured runs say about this bus.
///
/// Separate from the reads that took them so the verdict is assertable without
/// running a bus: the case exists to fail, and a verdict wired to the wrong
/// budget or to neither count would report a pass on a bus that overruns by
/// several times over, which is exactly the reading a person is waiting for.
/// Both distributions are printed whichever way it went.
fn timing_verdict(unicast: &TimingRun, grouped: &TimingRun, short: usize) -> CaseResult {
    let detail = format!("{unicast}; {grouped}");
    let over = unicast.overruns() + grouped.overruns();
    if over == 0 && short == 0 {
        CaseResult::pass(Case::BusExchangeTiming, detail)
    } else {
        CaseResult::fail(
            Case::BusExchangeTiming,
            format!(
                "{over} of {} exchanges ran past the cycle's budget and {short} grouped reads \
                 came back short; {detail}",
                unicast.len() + grouped.len()
            ),
        )
    }
}

/// The band a resting servo's present temperature has to read inside, degrees
/// Celsius.
///
/// Not a thermal limit — the servo's own temperature-limit register is that, and
/// this platform provisions it. This is the band that makes the temperature case
/// an assertion rather than a printout: a machine at rest in a room is inside
/// it, a servo that has been working hard is at the top of it, and a byte that is
/// not degrees at all — a zero from a register nothing answered, a raw tick count
/// — is outside it.
///
/// Wide on purpose: an unexpected value gets a person's review before this range
/// moves (bring-up rule, `CLAUDE.md`).
const RESTING_TEMP_BAND_C: core::ops::RangeInclusive<u8> = 5..=55;

/// The counts an antenna at rest may read: the turn a fold leaves, and half a
/// turn of slack on each side of it.
///
/// The slack is where a session's wind-down rests. Stow's count
/// representatives — a few counts above zero on one antenna and a few below on
/// the other — sit on the fold turn's own boundary, so a unit read after a
/// session rather than after a power cycle lands either side of it for a fully
/// explained reason. Half a turn each way is generous room around that and
/// still less than the 8250 counts of the recorded 545° anomaly, which is what
/// the case exists to catch.
const FOLD_WINDOW: core::ops::Range<i32> =
    (-COUNTS_PER_REV / 2)..(COUNTS_PER_REV + COUNTS_PER_REV / 2);

/// The slack every leg-fence bound is allowed, counts.
///
/// One count of rounding, and one only. The servo's window and the host's are
/// two descriptions of one physical limit reached by different arithmetic — a
/// provisioning tool wrote the counts, and this crate converts radians to counts
/// by rounding to nearest — so a bound landing a count either side of where the
/// conversion places it is agreement. Anything wider is two mechanisms guarding
/// different regions.
const FENCE_SLACK_COUNTS: i32 = 1;

/// The legs' travel fences as the provision sweep read them: per leg, the lower
/// bound then the upper, absent for a bound the sweep did not reach.
///
/// Carried from the sweep rather than read again. The sweep already reads both
/// limit registers off every servo, and a bus is serial and half-duplex: a
/// second read costs twelve round trips on the pass an operator runs
/// repeatedly, and — worse — two readings of one cell in one run can disagree,
/// which would put a passing sweep line and a failing fence line about the same
/// register in one report with nothing saying which is the truth.
type LegFences = [[Option<Value>; 2]; LEG_COUNT];

/// Which bound of a [`LegFences`] row a register is, if it is one.
const fn fence_bound_index(reg: RegId) -> Option<usize> {
    match reg {
        RegId::MinPositionLimit => Some(0),
        RegId::MaxPositionLimit => Some(1),
        _ => None,
    }
}

/// A travel window, lower then upper, in both units.
///
/// Both, because the two records this case compares are written in different
/// ones: a provisioning table is counts and an envelope is radians, and a
/// person reading a disagreement needs the pair.
struct Window([i32; 2]);

impl fmt::Display for Window {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [lower, upper] = self.0;
        write!(
            f,
            "[{lower} {upper}] counts / [{:.3} {:.3}]°",
            counts_to_rad(lower).to_degrees(),
            counts_to_rad(upper).to_degrees()
        )
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

/// Servo ids as a refusal names them: `11, 12, 17`.
fn ids_listed(ids: &[u8]) -> String {
    ids.iter().map(u8::to_string).collect::<Vec<_>>().join(", ")
}

/// A second value a provisioned register legitimately holds at rest, beside the
/// baseline [`ProvisionTable`] checks it against.
///
/// Lives here rather than in the compare loop so the loop stays
/// register-agnostic.
///
/// A register in this list is expected to read one value or the other on *every*
/// servo. A mix of the two is nothing at rest left behind, so it fails.
#[derive(Clone, Debug)]
struct RestingAlternate {
    /// The register whose second reading this is.
    reg: RegId,
    /// The value that reading holds.
    value: Value,
    /// What writes it, as a refusal names it: "expected 0 or the 10 <this>".
    written_by: &'static str,
    /// What every servo at the provisioned baseline says about the machine.
    all_baseline: &'static str,
    /// What every servo at [`RestingAlternate::value`] says about it.
    all_alternate: &'static str,
    /// What a mix of the two says — the text of the failure it produces.
    mixed: &'static str,
}

/// How many servos read each of a [`RestingAlternate`]'s two accepted values,
/// and which.
#[derive(Debug, Default)]
struct AlternateTally {
    baseline: Vec<u8>,
    alternate: Vec<u8>,
}

/// Everything the read-only registry needs that is not the machine.
///
/// Built from a configuration file, which records no datum: this registry is
/// what establishes the datum, by reading the offsets back. Nothing here is a
/// value the registry could send: the only wire traffic it produces is pings
/// and register reads.
#[derive(Clone, Debug)]
pub struct Registry {
    device: String,
    ids: [u8; ROW_COUNT],
    timing: BusTiming,
    map: ServoMap,
    expected: ProvisionTable,
    /// The registers with a second legitimate reading at rest. One entry today,
    /// the Bus Watchdog.
    resting: Vec<RestingAlternate>,
    geom: HeadGeometry,
    fk: FkOptions,
    env: EnvelopeConfig,
    min_arm_voltage: f64,
    goal_shadow_tolerance: f64,
}

impl Registry {
    /// The registry a configuration describes.
    ///
    /// Only the tables the read-only half needs are converted.
    pub fn from_config(cfg: &BenchConfig) -> Result<Self, ConfigError> {
        let ids = cfg.servo_ids()?;
        Ok(Self {
            device: cfg.bus.device.clone(),
            ids,
            timing: cfg.bus_timing()?,
            map: ServoMap::new(ids),
            expected: cfg.provision_table(),
            // The watchdog is the one provisioned register a motion session
            // writes and nothing clears short of power-on, so at rest it reads
            // either the provisioned baseline or what that session armed. A
            // session arms all nine; the bench's own `watchdog` command arms
            // one servo and disarms it again, so a mix is that disarm having
            // failed, or a session's commissioning sweep stopping part-way.
            resting: vec![RestingAlternate {
                reg: RegId::BusWatchdog,
                value: value::u8(cfg.resting_bus_watchdog()?),
                written_by: "a session arms",
                all_baseline: "no session since power-on",
                all_alternate: "a session has run this power cycle",
                mixed: "nothing at rest leaves the register in two states: a session that armed \
                        part of the roster, or a `watchdog` command whose disarm did not take",
            }],
            geom: HeadGeometry::default(),
            fk: FkOptions::default(),
            // The defaults and not a table in this file: the fence case compares
            // the servos against the envelope the commanding host actually runs,
            // and the commanding host is the cog system, which builds its arm
            // configuration over `EnvelopeConfig::default()`. A copy here would
            // let the bench agree with itself about a machine nothing else
            // agrees with.
            env: EnvelopeConfig::default(),
            // Checked here rather than inherited: this is the only consumer of
            // the figure, so nothing upstream of it runs the gate. Without it a
            // non-positive floor would let the voltage case pass a dead rail
            // and write that pass into the record a person reviews.
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
        let fences = self.provision(&mut bus, report);
        self.leg_fence(&fences, report);
        self.voltage(&mut bus, report);
        self.temperature(&mut bus, report);
        self.health(&mut bus, report);
        self.bus_exchange_timing(&mut bus, report);
        let Some(counts) = self.rest_pose(&mut bus, report) else {
            return;
        };
        self.goal_shadow(&mut bus, report);
        self.datum(&mut bus, report);
        self.antenna_fold(&counts, report);
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
    ///
    /// The legs' two position limits are handed back as well as reported: they
    /// are what [`Registry::leg_fence`] judges, and this is the pass that reads
    /// them. A sweep that stopped early hands back what it got to that point,
    /// and the fence case names the bound it was left without.
    fn provision<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) -> LegFences {
        let mut checked = 0usize;
        let mut recorded = 0usize;
        let mut wrong = Vec::new();
        let mut readings = Vec::new();
        let mut tallies: Vec<AlternateTally> = self
            .resting
            .iter()
            .map(|_| AlternateTally::default())
            .collect();
        let mut fences: LegFences = [[None; 2]; LEG_COUNT];

        for reg in reg::named() {
            let Some(column) = ProvisionTable::column(reg) else {
                continue;
            };
            let mut row_values = Vec::with_capacity(ROW_COUNT);
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
                        return fences;
                    }
                };
                read_any = true;
                if let (Some(bound), Some(leg)) = (
                    fence_bound_index(reg),
                    leg_index(ROWS[row]).map(usize::from),
                ) {
                    fences[leg][bound] = Some(value);
                }
                row_values.push(Shown(value).to_string());
                match expect {
                    ProvisionExpect::Check(expected) => {
                        checked += 1;
                        // The refusal names only the states this register has,
                        // so a value that is another register's alternate is
                        // never offered as if it were this one's.
                        let alternate = self
                            .resting
                            .iter()
                            .position(|entry| entry.reg == reg)
                            .map(|index| (index, &self.resting[index]));
                        match alternate {
                            Some((index, entry)) if value == entry.value => {
                                tallies[index].alternate.push(*id);
                            }
                            Some((index, _)) if value == expected => {
                                tallies[index].baseline.push(*id);
                            }
                            Some((_, entry)) => wrong.push(format!(
                                "servo {id} {}: expected {} or the {} {}, read {}",
                                RegName(reg),
                                Shown(expected),
                                Shown(entry.value),
                                entry.written_by,
                                Shown(value)
                            )),
                            None if value != expected => wrong.push(format!(
                                "servo {id} {}: expected {}, read {}",
                                RegName(reg),
                                Shown(expected),
                                Shown(value)
                            )),
                            None => {}
                        }
                    }
                    ProvisionExpect::Record => recorded += 1,
                    ProvisionExpect::Skip => {}
                }
            }
            if read_any {
                readings.push(format!("{} [{}]", RegName(reg), row_values.join(" ")));
            }
        }

        // Which of its two states a resting-alternate register was in is a
        // reading a person wants beside every other register on the sweep: for
        // the watchdog it is the whole record of whether this unit has run a
        // session since it was last powered on. A roster split across both
        // states records nothing of the sort, and is a failure.
        let mut states = String::new();
        for (entry, tally) in self.resting.iter().zip(&tallies) {
            match (tally.baseline.as_slice(), tally.alternate.as_slice()) {
                ([], []) => {}
                (baseline, []) => states.push_str(&format!(
                    "; {} at the provisioned baseline on {} servos: {}",
                    RegName(entry.reg),
                    baseline.len(),
                    entry.all_baseline
                )),
                ([], alternate) => states.push_str(&format!(
                    "; {} at the {} {} on {} servos: {}",
                    RegName(entry.reg),
                    Shown(entry.value),
                    entry.written_by,
                    alternate.len(),
                    entry.all_alternate
                )),
                (baseline, alternate) => wrong.push(format!(
                    "{}: at the provisioned baseline on servos {} and at the {} {} on servos {} \
                     — {}",
                    RegName(entry.reg),
                    ids_listed(baseline),
                    Shown(entry.value),
                    entry.written_by,
                    ids_listed(alternate),
                    entry.mixed
                )),
            }
        }
        let summary = format!(
            "{checked} checked, {recorded} recorded{states}: {}",
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
        fences
    }

    /// Each leg servo's own travel fence, against the envelope the commanding
    /// host runs.
    ///
    /// Two-sided, because the two records bracket one physical limit from
    /// opposite directions. The fence has to *contain* the window
    /// [`leg_windows`] derives — that window sits [`WINDOW_INSET_DEG`] inside
    /// the envelope, so exact equality is impossible by construction and
    /// containment is the right lower check — and it must not reach *past* the
    /// envelope's own window. A fence wider than the envelope is the same
    /// provisioning error in the direction that silently weakens the
    /// servo-side backstop, which is the one refusal left on the far side of
    /// the driver's seam, so it fails as loudly as a narrow one.
    ///
    /// Judged against the servos' actual limits, not the file's: a
    /// mis-provisioned unit is exactly what this catches. It gates nothing.
    ///
    /// The readings come from the provision sweep, which read the same two
    /// registers a moment earlier. Nothing here touches the bus: one read of a
    /// cell per pass, so a disagreement between two readings of it cannot be
    /// what a report is about.
    fn leg_fence(&self, fences: &LegFences, report: &mut Report) {
        let host = leg_windows(&self.env);
        let mut readings = Vec::with_capacity(LEG_COUNT);
        let mut wrong = Vec::new();

        for (row, joint) in ROWS.into_iter().enumerate() {
            let Some(leg) = leg_index(joint).map(usize::from) else {
                continue;
            };
            let fence = match self.fence_at(row, &fences[leg]) {
                Ok(fence) => fence,
                Err(detail) => {
                    report.push(CaseResult::fail(Case::LegFence, detail));
                    return;
                }
            };
            let (env_lower, env_upper) = self.env.crank_windows[leg];
            let (host_lower, host_upper) = host[leg];
            let placed = [
                self.counts_at(row, env_lower),
                self.counts_at(row, env_upper),
                self.counts_at(row, host_lower),
                self.counts_at(row, host_upper),
            ];
            let [env_low, env_high, host_low, host_high] = match placed {
                [Ok(a), Ok(b), Ok(c), Ok(d)] => [a, b, c, d],
                _ => {
                    let detail = placed
                        .into_iter()
                        .filter_map(Result::err)
                        .collect::<Vec<String>>()
                        .join("; ");
                    report.push(CaseResult::fail(Case::LegFence, detail));
                    return;
                }
            };
            let envelope = [env_low, env_high];
            readings.push(format!(
                "leg {leg}: servo {} against envelope {}",
                Window(fence),
                Window(envelope)
            ));

            // Four bounds and not two: each end of the fence is checked against
            // the window it must contain and against the window it must not
            // leave, and the two answers are different failures.
            let bounds = [
                (
                    "lower",
                    fence[0] <= host_low + FENCE_SLACK_COUNTS,
                    "starts inside the window the host commands in",
                ),
                (
                    "lower",
                    fence[0] >= env_low - FENCE_SLACK_COUNTS,
                    "starts below the envelope's own window",
                ),
                (
                    "upper",
                    fence[1] >= host_high - FENCE_SLACK_COUNTS,
                    "ends inside the window the host commands in",
                ),
                (
                    "upper",
                    fence[1] <= env_high + FENCE_SLACK_COUNTS,
                    "ends above the envelope's own window",
                ),
            ];
            for (bound, holds, complaint) in bounds {
                if !holds {
                    wrong.push(format!(
                        "leg {leg} {bound} bound {complaint}: servo {}, envelope {}",
                        Window(fence),
                        Window(envelope)
                    ));
                }
            }
        }

        let detail = format!(
            "{}; slack {FENCE_SLACK_COUNTS} count, host window inset \
             {WINDOW_INSET_DEG}°",
            readings.join("; ")
        );
        if wrong.is_empty() {
            report.push(CaseResult::pass(Case::LegFence, detail));
        } else {
            report.push(CaseResult::fail(
                Case::LegFence,
                format!("{}; {detail}", wrong.join("; ")),
            ));
        }
    }

    /// One leg servo's provisioned travel window, counts, lower then upper.
    fn fence_at(&self, row: usize, read: &[Option<Value>; 2]) -> Result<[i32; 2], String> {
        Ok([
            self.fence_bound(row, RegId::MinPositionLimit, read[0])?,
            self.fence_bound(row, RegId::MaxPositionLimit, read[1])?,
        ])
    }

    /// One bound of one servo's travel window, as the signed count the
    /// conversions take.
    ///
    /// One refusal for the reading and one for the number, and no third answer
    /// to "what width is this register": the sweep read it through the named
    /// vocabulary, so a value that is not the register's own shape or not a
    /// count this comparison can take is one thing to say — the register holds
    /// something no fence can be judged from.
    fn fence_bound(&self, row: usize, reg: RegId, read: Option<Value>) -> Result<i32, String> {
        let value = read.ok_or_else(|| {
            format!(
                "servo {} {}: the provisioned registers sweep did not read it, so there is no \
                 fence to judge",
                self.ids[row],
                RegName(reg)
            )
        })?;
        value
            .as_u32()
            .and_then(|counts| i32::try_from(counts).ok())
            .ok_or_else(|| {
                format!(
                    "servo {} {}: {} is no count this comparison takes",
                    self.ids[row],
                    RegName(reg),
                    Shown(value)
                )
            })
    }

    /// A crank angle as the count a fence bound is compared against.
    fn counts_at(&self, row: usize, rad: f64) -> Result<i32, String> {
        self.map.goal_counts(row, rad).map_err(|error| {
            format!(
                "servo {}: {:.3}° is no count ({error})",
                self.ids[row],
                rad.to_degrees()
            )
        })
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

    /// Every servo's present temperature, against the band a resting machine
    /// reads in.
    ///
    /// The band is what makes the assertion worth anything: a room-temperature
    /// machine is comfortably inside it, and a byte that is not degrees at all
    /// reads outside it. A reading outside the band gets a person before it gets
    /// a wider band, per the bring-up rule (`CLAUDE.md`). Nothing gates on this
    /// case: it is a diagnostic and a regression guard, like the rest of the
    /// registry.
    fn temperature<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let degrees = match self.sweep_u8(bus, RegId::PresentTemperature) {
            Ok(degrees) => degrees,
            Err(detail) => {
                report.push(CaseResult::fail(Case::Temperature, detail));
                return;
            }
        };
        let outside: Vec<String> = self
            .ids
            .iter()
            .zip(degrees.iter())
            .filter(|(_, reading)| !RESTING_TEMP_BAND_C.contains(reading))
            .map(|(id, reading)| format!("{id} at {reading} C"))
            .collect();
        let detail = format!(
            "{degrees:?} C against {}..={} C",
            RESTING_TEMP_BAND_C.start(),
            RESTING_TEMP_BAND_C.end()
        );
        if outside.is_empty() {
            report.push(CaseResult::pass(Case::Temperature, detail));
        } else {
            report.push(CaseResult::fail(
                Case::Temperature,
                format!("outside the band: {}; {detail}", outside.join(", ")),
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

    /// What an exchange on this bus actually costs, against what a control
    /// cycle budgets for one.
    ///
    /// Two kinds, because the cycle runs both and they are budgeted apart: the
    /// unicast register read the out-of-band slot spends its transactions on,
    /// and the grouped read that gathers nine positions and is the cycle's most
    /// expensive exchange. Every exchange is timed on both sides of its write,
    /// so an overrun says whether the host was waiting for its own bytes to
    /// leave or for a servo to answer.
    ///
    /// The exchanges run under the *configured* deadline, which is the generous
    /// one, and are judged against the cycle's. A measurement taken under the
    /// deadline it is judged by could only ever report the deadline.
    ///
    /// Nothing is retried: a retry's second attempt would replace the reading
    /// the case exists to take. A servo that will not answer fails the case by
    /// name, and a grouped read that came back short says which slots.
    fn bus_exchange_timing<P: BusPort>(&self, bus: &mut Bus<P>, report: &mut Report) {
        let budget = BusTiming {
            host_allowance: CYCLE_HOST_ALLOWANCE,
            ..self.timing
        };
        let error_status = named_reg(RegId::HardwareErrorStatus);
        let position = named_reg(RegId::PresentPosition);
        let id = self.ids[0];

        let mut unicast = TimingRun::new(
            "unicast read",
            budget.read_reg_bound(usize::from(error_status.len)),
        );
        for _ in 0..TIMING_EXCHANGES {
            if let Err(error) = bus.read_reg(id, error_status) {
                report.push(CaseResult::fail(
                    Case::BusExchangeTiming,
                    format!(
                        "servo {id} stopped answering after {} reads: {error}",
                        unicast.len()
                    ),
                ));
                return;
            }
            unicast.note(bus.last_spans());
        }

        let mut grouped = TimingRun::new(
            "grouped read",
            budget.sync_read_bound(self.ids.len(), usize::from(position.len)),
        );
        let mut outcome = SyncReadOutcome::new();
        let mut short = 0;
        for _ in 0..TIMING_EXCHANGES {
            if let Err(error) = bus.sync_read(&self.ids, position, &mut outcome) {
                report.push(CaseResult::fail(
                    Case::BusExchangeTiming,
                    format!(
                        "the grouped read failed after {} of them: {error}",
                        grouped.len()
                    ),
                ));
                return;
            }
            grouped.note(bus.last_spans());
            if !outcome.all_ok() {
                short += 1;
            }
        }

        report.push(timing_verdict(&unicast, &grouped, short));
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
    ) -> Option<[i32; ROW_COUNT]> {
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

        let mut readings = Vec::with_capacity(ROW_COUNT);
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
            ROW_COUNT - holding,
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

    /// Each antenna's resting count against the turn a boot fold leaves it in,
    /// with half a turn of slack either side for where a session's wind-down
    /// rests.
    ///
    /// The antennas are the two joints in extended position mode, so their
    /// position register counts turns rather than wrapping: a sweep past the
    /// count frame's boundary keeps counting, and the register can stand
    /// hundreds of turns from zero. Powering the servos up folds that count
    /// back into one turn, and so does the `reboot` command — two power-on
    /// observations and the reboot's own behaviour are what that is measured
    /// on. Everything downstream assumes it: a resting antenna's converted
    /// angle is where the antenna physically is exactly when the fold has
    /// happened, and a sweep planned from a count several turns out is a sweep
    /// several turns long.
    ///
    /// The slack is the wind-down, not a travel cap — nothing caps an antenna
    /// short of representability. A session leaves its antennas at stow, whose
    /// count representatives sit within a few counts of the fold turn's own
    /// boundary, so a unit read after a session rather than after a power cycle
    /// legitimately rests a little either side of it. Half a turn each way is
    /// generous slack around that rest, and the downstream requirement — a
    /// resting count that is not several turns out — survives it.
    ///
    /// Read off the resting-pose sweep rather than asked for again — this is a
    /// second question about the counts that case already recorded, and the
    /// antennas are free rotors nothing is holding still between two reads.
    ///
    /// One reading of 545° immediately after a hard power cycle is on record
    /// and unexplained. 545° is 8250 counts, more than half a turn past this
    /// window, so if that observation recurs this case still fails and it is
    /// still a person's to look at: the count frame is the datum every antenna
    /// command is planned in, and a bound widened to admit an unfolded reading
    /// would launder the anomaly into accepted behaviour. Normalising the count
    /// modulo a turn before comparing would be exactly that laundering, and is
    /// not done.
    fn antenna_fold(&self, counts: &[i32; ROW_COUNT], report: &mut Report) {
        let mut readings = Vec::new();
        let mut unfolded = Vec::new();
        for (row, joint) in ROWS.into_iter().enumerate() {
            if group_of(joint) != Some(JointGroup::Antennas) {
                continue;
            }
            let (id, count) = (self.ids[row], counts[row]);
            let degrees = counts_to_rad(count).to_degrees();
            readings.push(format!("{id}: {count} counts ({degrees:.3} deg)"));
            if !FOLD_WINDOW.contains(&count) {
                unfolded.push(format!(
                    "servo {id}: {count} counts, {degrees:.3} deg, is outside the turn a fold \
                     leaves and the half-turn of wind-down slack either side"
                ));
            }
        }

        let detail = format!(
            "against counts {}..{} — the {COUNTS_PER_REV}-count turn a fold leaves, with half a \
             turn of wind-down slack either side: {}",
            FOLD_WINDOW.start,
            FOLD_WINDOW.end,
            readings.join("; ")
        );
        if unfolded.is_empty() {
            report.push(CaseResult::pass(Case::AntennaFold, detail));
        } else {
            report.push(CaseResult::fail(
                Case::AntennaFold,
                format!("{}; {detail}", unfolded.join("; ")),
            ));
        }
    }

    /// The clearance the resting pose leaves from the linkage's singular
    /// configurations.
    fn rest_margins(&self, counts: &[i32; ROW_COUNT], report: &mut Report) {
        let mut joints = JointVector::default();
        // The joint layout is asked for rather than restated: each reading is
        // filed against the joint that bus row belongs to.
        for (row, (id, count)) in ROWS.into_iter().zip(counts.iter()).enumerate() {
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
    ///
    /// The read itself is `bare`'s, so the crate has one answer to "read one
    /// register from one servo"; what is added here is the string a report
    /// carries, which is this module's own boundary.
    fn read_value<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        row: usize,
        reg: RegId,
    ) -> Result<Value, String> {
        bare::read_value(bus, &self.map, row, reg).map_err(|error| error.to_string())
    }

    /// One register from one servo, as the bytes it holds.
    fn read_raw<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        row: usize,
        reg: RegId,
    ) -> Result<RawValue, String> {
        bare::read_raw(bus, &self.map, row, reg).map_err(|error| error.to_string())
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
    ) -> Result<[T; ROW_COUNT], String> {
        let mut out = [T::default(); ROW_COUNT];
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
    ) -> Result<[u8; ROW_COUNT], String> {
        self.sweep(bus, reg, RawValue::u8)
    }

    /// One two-byte register from all nine servos.
    fn sweep_u16<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
    ) -> Result<[u16; ROW_COUNT], String> {
        self.sweep(bus, reg, RawValue::u16)
    }

    /// One four-byte signed register from all nine servos.
    fn sweep_i32<P: BusPort>(
        &self,
        bus: &mut Bus<P>,
        reg: RegId,
    ) -> Result<[i32; ROW_COUNT], String> {
        self.sweep(bus, reg, RawValue::i32)
    }

    /// A reply the register's width does not fit. Unreachable while the control
    /// table and the transaction layer's length check agree; reported rather
    /// than assumed away because the case's job is saying what it saw.
    fn width_detail(&self, row: usize, reg: RegId, raw: &RawValue) -> String {
        format!(
            "servo {} {}: {} bytes are not the width this register reads as",
            self.ids[row],
            RegName(reg),
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
    use dxl_proto::frame::{INST_PING, INST_READ, INST_SYNC_READ};
    use reachy_motion::joints::Name;

    use super::*;
    use reachy_bus::named_reg;

    use crate::testutil::{FakeMachine, Spy, example_config, machine_at, rest_legs, stow_legs};

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

    /// A watched run and, with it, every servo a grouped read named.
    type WatchedGrouped = (Report, Vec<(u8, u8)>, Vec<(u8, u16)>, Vec<u8>);

    /// The same run, carrying the register each unicast read asked for as well
    /// as the instructions — which is the only record of *which* register a case
    /// put on the wire, and how often.
    fn run_watched(cfg: &BenchConfig, machine: FakeMachine) -> Watched {
        let (report, instructions, asked, _) = run_watched_grouped(cfg, machine);
        (report, instructions, asked)
    }

    /// The same run again, carrying the servos the grouped reads named — the
    /// only record of who a broadcast frame actually asked.
    fn run_watched_grouped(cfg: &BenchConfig, machine: FakeMachine) -> WatchedGrouped {
        let registry = Registry::from_config(cfg).expect("the configuration converts");
        let spy = Spy::new(machine);
        let log = spy.log();
        let reads = spy.reads();
        let sync_ids = spy.sync_ids();
        let mut report = Report::new();
        registry.run(Ok::<Spy, String>(spy), &mut report);
        let instructions = log.borrow().clone();
        let asked = reads.borrow().clone();
        let grouped = sync_ids.borrow().clone();
        (report, instructions, asked, grouped)
    }

    /// What one case said about itself.
    ///
    /// A case's own detail and not the printed report: several cases read the
    /// same registers, so an assertion against the whole report can be satisfied
    /// by a neighbour's line while the case under test says nothing at all.
    fn detail_of(report: &Report, case: Case) -> String {
        report
            .results()
            .iter()
            .find(|result| result.case == case)
            .map(|result| result.detail.clone())
            .unwrap_or_else(|| panic!("{case:?} left no result in {report}"))
    }

    /// What a run over the fake machine may assert about the timing case: that
    /// it ran, that it printed all three distributions over the exchanges it
    /// says it took, and that anything it failed on was the wall-clock budget
    /// and nothing else.
    ///
    /// Not the pass/fail verdict. That verdict is a statement about a machine —
    /// over a fake port the medians are microseconds, but the maximum belongs
    /// to the host's scheduler, and one descheduling of the test thread in 400
    /// exchanges runs past a serial bus's budget. The verdict arithmetic is
    /// pinned deterministically over synthetic spans in `timing_verdict`'s own
    /// tests, and the pass direction on a real bus is the hardware registry's
    /// to hold.
    fn assert_timing_case_ran_clean(report: &Report) {
        let outcome = report.outcome(Case::BusExchangeTiming);
        assert_ne!(outcome, Outcome::NotRun, "the timing case ran: {report}");
        let detail = detail_of(report, Case::BusExchangeTiming);
        for expected in [
            format!("unicast read x{TIMING_EXCHANGES}"),
            format!("grouped read x{TIMING_EXCHANGES}"),
            "total ".to_string(),
            "write ".to_string(),
            "wait ".to_string(),
        ] {
            assert!(detail.contains(&expected), "{detail}");
        }
        if outcome != Outcome::Pass {
            assert!(
                detail.contains("0 grouped reads came back short"),
                "a fake port answers every slot: {detail}"
            );
            assert!(
                !detail.contains("servo "),
                "a fake port's servos all answer: {detail}"
            );
        }
    }

    /// The bus watchdog reads one of two things on a machine at rest: the
    /// provisioned zero on a unit power-cycled since its last session, and the
    /// timeout a session arms on one that has run since. Both pass, and the
    /// sweep's line says which was read — that is the whole record of whether a
    /// session has run this power cycle.
    #[test]
    fn the_watchdog_row_accepts_the_baseline_and_the_value_a_session_arms() {
        let cfg = example_config();
        let (report, _) = run(&cfg, machine_at(&cfg, &stow_legs()));
        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Pass);
        assert!(
            report.to_string().contains("no session since power-on"),
            "{report}"
        );

        let mut machine = machine_at(&cfg, &stow_legs());
        for id in cfg.servo_ids().expect("the roster") {
            machine.set(
                id,
                named_reg(RegId::BusWatchdog),
                &[cfg.provision.bus_watchdog_armed],
            );
        }
        let (report, _) = run(&cfg, machine);
        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Pass);
        assert!(
            report
                .to_string()
                .contains("a session has run this power cycle"),
            "{report}"
        );
    }

    /// Both accepted readings are the *roster's*, not a servo's. A unit split
    /// across them is neither state and records nothing about whether a session
    /// has run: a session's commissioning sweep arms all nine, power-on clears
    /// all nine, and the one thing that arms a single servo — the bench's own
    /// `watchdog` command — disarms it again on the way out. A mix is that
    /// disarm having failed or an arm that stopped part-way, which is exactly
    /// the state a person needs to see.
    #[test]
    fn a_roster_split_across_both_rest_states_fails_naming_the_servos() {
        let cfg = example_config();
        let ids = cfg.servo_ids().expect("the roster");
        let mut machine = machine_at(&cfg, &stow_legs());
        for id in ids.iter().skip(1) {
            machine.set(
                *id,
                named_reg(RegId::BusWatchdog),
                &[cfg.provision.bus_watchdog_armed],
            );
        }
        let (report, _) = run(&cfg, machine);

        assert_eq!(
            report.outcome(Case::ProvisionSweep),
            Outcome::Fail,
            "one servo at the baseline and eight armed is neither rest state"
        );
        let printed = report.to_string();
        assert!(
            printed.contains(&format!("on servos {}", ids[0])),
            "the odd servo out is named: {printed}"
        );
        assert!(
            printed.contains("whose disarm did not take"),
            "and the line says what leaves a roster split: {printed}"
        );
        assert!(
            !printed.contains("a session has run this power cycle"),
            "a split roster is not the record that a session ran: {printed}"
        );
    }

    /// Anything else in the register fails the sweep by name, a latched trip
    /// (0xFF) among them: a watchdog that tripped and stayed tripped across the
    /// end of a session is an unexplained event, not a state something left
    /// behind on purpose.
    #[test]
    fn the_watchdog_row_fails_on_any_third_reading() {
        let cfg = example_config();
        for reading in [0xFFu8, 5] {
            let mut machine = machine_at(&cfg, &stow_legs());
            machine.set(11, named_reg(RegId::BusWatchdog), &[reading]);
            let (report, _) = run(&cfg, machine);

            assert_eq!(
                report.outcome(Case::ProvisionSweep),
                Outcome::Fail,
                "{reading} is neither the baseline nor what a session arms"
            );
            let printed = report.to_string();
            assert!(printed.contains("servo 11"), "{printed}");
            assert!(
                printed.contains("a session arms"),
                "the refusal names both accepted states: {printed}"
            );
        }
    }

    /// Only the watchdog has a second accepted state. Every other provisioned
    /// register refuses a wrong reading with a line naming the one value it
    /// wanted — the reading a person is handed as the evidence, so it must not
    /// offer an alternative that register would have failed on anyway. The
    /// value a session arms the watchdog with is the sharp case: read back from
    /// Drive Mode it is simply wrong, and the refusal has to say so.
    #[test]
    fn a_non_watchdog_register_fails_naming_only_its_own_expectation() {
        let cfg = example_config();
        let armed = cfg.provision.bus_watchdog_armed;
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(11, named_reg(RegId::DriveMode), &[armed]);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 11"), "{printed}");
        assert!(
            !printed.contains("a session arms"),
            "the watchdog's second state is not on offer for Drive Mode: {printed}"
        );
    }

    /// A machine holding what it should, resting at the stow pose, passes every
    /// case the registry has — the clearance among them, since the floor is a
    /// reviewed number and a correct machine clears it.
    ///
    /// Every case but the timing one, which judges wall-clock spans against a
    /// serial bus's budget and so answers about the host running the test
    /// rather than about the machine; it is asserted to have run clean instead.
    #[test]
    fn a_correct_machine_passes_every_case() {
        let cfg = example_config();
        let machine = machine_at(&cfg, &stow_legs());
        let (report, _) = run(&cfg, machine);

        for case in Case::ALL {
            if case == Case::BusExchangeTiming {
                assert_timing_case_ran_clean(&report);
                continue;
            }
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
        assert!(
            report
                .results()
                .iter()
                .filter(|result| result.case != Case::BusExchangeTiming)
                .all(|result| result.outcome.passed()),
            "every case but the timing one passed: {report}"
        );

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
    ///
    /// The grouped read is the one request that does not wear one address: it
    /// goes to the broadcast id and carries its roster in its parameters. It is
    /// still a read and still asks only configured servos, which is what is
    /// asserted of it here — the frame's own id is exempted, the servos it
    /// names are not.
    #[test]
    fn the_registry_only_ever_pings_and_reads_its_own_roster() {
        let cfg = example_config();
        let machine = machine_at(&cfg, &stow_legs());
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let (_, instructions, _, grouped) = run_watched_grouped(&cfg, machine);

        assert!(!instructions.is_empty());
        let mut seen = Vec::new();
        for (id, instruction) in &instructions {
            assert!(
                *instruction == INST_PING
                    || *instruction == INST_READ
                    || *instruction == INST_SYNC_READ,
                "servo {id} was sent instruction {instruction:#04x}"
            );
            if *instruction == INST_SYNC_READ {
                assert_eq!(
                    *id,
                    dxl_proto::BROADCAST_ID,
                    "a grouped read is addressed to the broadcast id and to nothing else"
                );
                continue;
            }
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

        assert!(!grouped.is_empty(), "the timing case runs grouped reads");
        for id in &grouped {
            assert!(
                ids.contains(id),
                "a grouped read named {id}, not on the roster"
            );
        }
    }

    /// A clearance floor the configuration cannot stand behind refuses before
    /// the registry is built, rather than being carried into the voltage case.
    ///
    /// Nothing upstream of the registry judges this figure, so the positivity
    /// gate is run where it is consumed. Without it a negative floor passes a
    /// dead rail and writes that pass into the record a person reviews.
    #[test]
    fn an_arming_floor_that_is_not_a_voltage_refuses_the_registry() {
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY] {
            let mut cfg = example_config();
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

    /// A second rest state that cannot be told from the first refuses before
    /// the registry is built.
    ///
    /// The sweep matches the alternate before the baseline, so a configuration
    /// whose two accepted readings are the same value would tally a virgin,
    /// power-cycled unit as armed and print "a session has run this power
    /// cycle" over a machine that has run none — the one line the case exists
    /// to produce, inverted, on a green sweep. Refused rather than ordered
    /// around, because a split roster would be undetectable under it either
    /// way.
    #[test]
    fn a_rest_state_equal_to_the_baseline_refuses_the_registry() {
        let mut cfg = example_config();
        cfg.provision.bus_watchdog_armed = cfg.provision.bus_watchdog;
        let Err(refusal) = Registry::from_config(&cfg) else {
            panic!("the two rest states are one value, and a registry was built on it");
        };
        assert_eq!(
            refusal,
            ConfigError::RestingStatesCollide {
                alternate: "provision.bus_watchdog_armed",
                baseline: "provision.bus_watchdog",
                value: cfg.provision.bus_watchdog,
            },
            "{refusal}"
        );
    }

    /// The tripped reading is what the sweep fails on, so it cannot also be
    /// configured as a state the machine legitimately rests in.
    #[test]
    fn a_rest_state_that_is_the_tripped_reading_refuses_the_registry() {
        let mut cfg = example_config();
        cfg.provision.bus_watchdog_armed = crate::bare::WATCHDOG_LATCHED;
        let Err(refusal) = Registry::from_config(&cfg) else {
            panic!("a latched trip is not a rest state, and a registry accepted it as one");
        };
        assert_eq!(
            refusal,
            ConfigError::RestingStateIsATrip {
                key: "provision.bus_watchdog_armed",
                value: crate::bare::WATCHDOG_LATCHED,
            },
            "{refusal}"
        );
    }

    /// A port that does not open fails the first case and runs nothing after it.
    #[test]
    fn a_port_that_does_not_open_stops_the_run_at_the_first_case() {
        let cfg = example_config();
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
        let cfg = example_config();
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // Operating mode 0 on one servo: the reading that voids the servo-side
        // position envelope.
        machine.set(13, named_reg(RegId::OperatingMode), &[0]);
        machine.set(16, named_reg(RegId::TemperatureLimit), &[95]);
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

    /// A register a servo will not answer at all fails the sweep with the servo
    /// and the register in the text an operator reads.
    ///
    /// The read itself is `bare`'s and the string is this module's: what a
    /// failure prints comes from `BareError`'s own rendering, so the servo
    /// and the register have to survive that hand-off.
    #[test]
    fn a_register_a_servo_will_not_answer_fails_the_sweep_by_servo_and_register() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .mute
            .insert((13, named_reg(RegId::OperatingMode).addr), u32::MAX);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::ProvisionSweep), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 13"), "{printed}");
        assert!(printed.contains("operating mode"), "{printed}");
    }

    /// The timing case measures both kinds of exchange and says what each
    /// cost, whichever way the assertion went.
    ///
    /// Over a fake port the medians are microseconds, the maximum is the host
    /// scheduler's, and the pass/fail verdict is therefore not this test's to
    /// assert — the deterministic verdict arithmetic is pinned in
    /// `timing_verdict`'s own tests. What is asserted here is that the case ran
    /// the exchanges it says it ran and printed all three distributions,
    /// because the printed breakdown is the whole product of the case on a run
    /// where it fails.
    #[test]
    fn the_timing_case_prints_what_each_kind_of_exchange_cost() {
        let cfg = example_config();
        let machine = machine_at(&cfg, &stow_legs());
        let (report, _, reads) = run_watched(&cfg, machine);

        assert_timing_case_ran_clean(&report);
        let detail = detail_of(&report, Case::BusExchangeTiming);

        let status = named_reg(RegId::HardwareErrorStatus).addr;
        let asked = reads
            .iter()
            .filter(|(id, addr)| *id == 10 && *addr == status)
            .count();
        assert!(
            asked >= TIMING_EXCHANGES,
            "the case reads servo 10's error status {TIMING_EXCHANGES} times; saw {asked}"
        );

        // The budget printed is the cycle's, not the configured deadline the
        // exchanges ran under: judged by the generous one the case could never
        // fail, whatever the bus cost.
        let registry = Registry::from_config(&cfg).expect("the configuration converts");
        let cycle = BusTiming {
            host_allowance: CYCLE_HOST_ALLOWANCE,
            ..registry.timing
        };
        let width = usize::from(named_reg(RegId::HardwareErrorStatus).len);
        assert!(
            detail.contains(&format!(
                "against {} ms",
                millis(cycle.read_reg_bound(width))
            )),
            "{detail}"
        );
        assert!(
            !detail.contains(&format!(
                "against {} ms",
                millis(registry.timing.read_reg_bound(width))
            )),
            "the configured allowance is not what an exchange is judged by: {detail}"
        );
    }

    /// A grouped read that comes back missing a servo fails the timing case and
    /// says so, rather than passing on a distribution over partial readings.
    #[test]
    fn a_grouped_read_that_comes_back_short_fails_the_timing_case() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        let registry = Registry::from_config(&cfg).expect("the configuration converts");
        machine
            .deaf
            .insert((registry.ids[4], named_reg(RegId::PresentPosition).addr));
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::BusExchangeTiming), Outcome::Fail);
        let detail = detail_of(&report, Case::BusExchangeTiming);
        assert!(detail.contains("came back short"), "{detail}");
        assert!(
            detail.contains(&format!("{TIMING_EXCHANGES} grouped reads came back short")),
            "every one of them was short: {detail}"
        );
    }

    /// A servo that stops answering fails the timing case by name rather than
    /// leaving a distribution over the exchanges that did answer.
    #[test]
    fn a_servo_that_stops_answering_fails_the_timing_case_by_name() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .deaf
            .insert((10, named_reg(RegId::HardwareErrorStatus).addr));
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::BusExchangeTiming), Outcome::Fail);
        let detail = detail_of(&report, Case::BusExchangeTiming);
        assert!(detail.contains("servo 10"), "{detail}");
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(17, named_reg(RegId::OperatingMode), &[3]);
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            15,
            named_reg(RegId::PresentInputVoltage),
            &45u16.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Voltage), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("15 at 4.5 V"), "{printed}");
        assert!(printed.contains("11.8"), "{printed}");
    }

    /// A temperature outside the resting band fails and names the servo, in both
    /// directions.
    ///
    /// Both, because the two ends catch different things: the top is a servo
    /// getting hot, and the bottom is a reading that is not degrees Celsius at
    /// all.
    #[test]
    fn a_temperature_outside_the_resting_band_fails_and_says_which_servo() {
        let cfg = example_config();
        let legs = stow_legs();

        let mut hot = machine_at(&cfg, &legs);
        hot.set(13, named_reg(RegId::PresentTemperature), &[90]);
        let (report, _) = run(&cfg, hot);
        assert_eq!(report.outcome(Case::Temperature), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("13 at 90 C"), "{printed}");
        assert!(
            printed.contains("5..=55 C"),
            "the band is printed: {printed}"
        );

        let mut cold = machine_at(&cfg, &legs);
        cold.set(13, named_reg(RegId::PresentTemperature), &[0]);
        let (report, _) = run(&cfg, cold);
        assert_eq!(report.outcome(Case::Temperature), Outcome::Fail);
        assert!(
            report.to_string().contains("13 at 0 C"),
            "the servo is named"
        );

        // And the fixture as this platform reads passes, so the cases above are
        // the reading and not the case.
        let (report, _) = run(&cfg, machine_at(&cfg, &legs));
        assert_eq!(report.outcome(Case::Temperature), Outcome::Pass);
    }

    /// The fences this platform provisions agree with the envelope the cog
    /// system commands through, and the reading is printed either way.
    ///
    /// The pass is the load-bearing half: the provisioned counts and
    /// `EnvelopeConfig::default()` are edited in different files by different
    /// people, and this is the only thing in the tree that compares them.
    #[test]
    fn the_provisioned_leg_fences_agree_with_the_envelope() {
        let cfg = example_config();
        let (report, _) = run(&cfg, machine_at(&cfg, &stow_legs()));

        assert_eq!(report.outcome(Case::LegFence), Outcome::Pass);
        let printed = report.to_string();
        assert!(
            printed.contains("leg 0: servo [1502 2958] counts"),
            "{printed}"
        );
        assert!(
            printed.contains("leg 5: servo [1138 2594] counts"),
            "{printed}"
        );
        assert!(
            printed.contains("slack 1 count"),
            "the slack is printed: {printed}"
        );
    }

    /// A fence narrower than the window the host commands in fails, naming the
    /// leg, the bound and both windows in both units.
    #[test]
    fn a_fence_inside_the_host_window_fails_and_says_which_bound() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // Leg 0 is servo 11, a hundred counts up from where it is provisioned:
        // a window the host would command straight through the bottom of.
        machine.set(
            11,
            named_reg(RegId::MinPositionLimit),
            &1602u32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::LegFence), Outcome::Fail);
        let printed = report.to_string();
        assert!(
            printed.contains("leg 0 lower bound starts inside the window the host commands in"),
            "{printed}"
        );
        assert!(
            printed.contains("servo [1602 2958] counts / [-39.199 79.980]°"),
            "the servo's window in both units: {printed}"
        );
        assert!(
            printed.contains("envelope [1502 2958] counts / [-47.988 79.980]°"),
            "the envelope's window in both units: {printed}"
        );
    }

    /// A fence wider than the envelope fails too — the direction that silently
    /// weakens the servo-side backstop rather than the one that refuses a
    /// commanded pose.
    #[test]
    fn a_fence_past_the_envelope_fails_and_says_which_bound() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            11,
            named_reg(RegId::MaxPositionLimit),
            &3058u32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::LegFence), Outcome::Fail);
        let printed = report.to_string();
        assert!(
            printed.contains("leg 0 upper bound ends above the envelope's own window"),
            "{printed}"
        );
        assert!(
            printed.contains("servo [1502 3058] counts"),
            "the servo's window: {printed}"
        );
    }

    /// One count either side of where the conversion places a bound is
    /// agreement, on both ends and in both directions.
    ///
    /// The tolerance is the case's whole tuning, and a polarity error in it
    /// would either refuse every unit or accept any fence at all. Both bounds
    /// are moved in both directions in one run.
    #[test]
    fn one_count_of_rounding_on_a_bound_is_agreement() {
        let cfg = example_config();
        for (lower, upper) in [(1501u32, 2959u32), (1503u32, 2957u32)] {
            let mut machine = machine_at(&cfg, &stow_legs());
            machine.set(11, named_reg(RegId::MinPositionLimit), &lower.to_le_bytes());
            machine.set(11, named_reg(RegId::MaxPositionLimit), &upper.to_le_bytes());
            let (report, _) = run(&cfg, machine);

            assert_eq!(
                report.outcome(Case::LegFence),
                Outcome::Pass,
                "[{lower} {upper}] is one count of rounding: {report}"
            );
        }

        // And two counts is not, in the direction the slack is widest.
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            11,
            named_reg(RegId::MinPositionLimit),
            &1500u32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);
        assert_eq!(report.outcome(Case::LegFence), Outcome::Fail);
        assert!(
            report
                .to_string()
                .contains("leg 0 lower bound starts below the envelope's own window"),
            "{report}"
        );
    }

    /// A leg whose fence register answers nothing fails the case naming the
    /// servo and the register, rather than reading as a window of zero.
    ///
    /// Asserted on this case's own detail rather than on the printed report:
    /// muting a provisioned register fails the sweep as well, and the sweep's
    /// line names the same servo and the same register — so a check against the
    /// whole report would pass with this case's detail empty, which is the one
    /// condition where the detail is all an operator has.
    #[test]
    fn a_fence_register_nothing_answers_fails_by_name() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .mute
            .insert((13, named_reg(RegId::MaxPositionLimit).addr), u32::MAX);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::LegFence), Outcome::Fail);
        let said = detail_of(&report, Case::LegFence);
        assert!(
            said.contains("servo 13 maximum position limit"),
            "the servo and the register: {said}"
        );
        assert!(
            said.contains("no fence to judge"),
            "and what that leaves the case with: {said}"
        );
    }

    /// A fence bound holding a number no count can be taken from fails by name
    /// rather than being wrapped into a plausible-looking window.
    ///
    /// The registers decode as unsigned and the comparison is signed, so a cell
    /// at or above 2^31 is a real reading this case cannot judge. Wrapping it
    /// instead would put a large negative bound against a real window and read
    /// as agreement — a mis-provisioned unit passing.
    #[test]
    fn a_fence_bound_no_count_can_take_fails_by_name() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            11,
            named_reg(RegId::MinPositionLimit),
            &0x8000_0000u32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::LegFence), Outcome::Fail);
        let said = detail_of(&report, Case::LegFence);
        assert!(
            said.contains("servo 11 minimum position limit"),
            "the servo and the register: {said}"
        );
        assert!(
            said.contains("is no count this comparison takes"),
            "and why the case cannot judge it: {said}"
        );
    }

    /// A servo that will not answer the temperature cell fails the temperature
    /// case, not some other case and not a case that never ran.
    ///
    /// The sweep's refusal is shared with every other register swept, so what is
    /// pinned here is which case it lands on: `NotRun` and `Fail` are different
    /// stories to an operator triaging a bus.
    #[test]
    fn a_temperature_nothing_answers_fails_that_case_by_name() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .mute
            .insert((13, named_reg(RegId::PresentTemperature).addr), u32::MAX);
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::Temperature), Outcome::Fail);
        let said = detail_of(&report, Case::Temperature);
        assert!(said.contains("13"), "the servo is named: {said}");
        assert!(
            said.contains("present temperature"),
            "the register is named: {said}"
        );
    }

    /// Every cell this registry reads, it reads once per pass.
    ///
    /// Asserted off the wire for the fence case in particular: it judges the
    /// limit registers the provision sweep already read, and a refactor that had
    /// it ask the bus itself would leave every other case in this file green
    /// while costing twelve round trips on the pass an operator runs repeatedly
    /// — and putting two readings of one cell in one report, with nothing saying
    /// which of them the verdict is about.
    #[test]
    fn the_fence_and_temperature_cells_are_read_once_each() {
        let cfg = example_config();
        let machine = machine_at(&cfg, &stow_legs());
        let (report, _, reads) = run_watched(&cfg, machine);

        assert_eq!(report.outcome(Case::LegFence), Outcome::Pass);
        assert_eq!(report.outcome(Case::Temperature), Outcome::Pass);
        for (row, id) in cfg
            .servo_ids()
            .expect("the roster is nine servos")
            .into_iter()
            .enumerate()
        {
            let asked = |reg: RegId| {
                reads
                    .iter()
                    .filter(|(servo, addr)| *servo == id && *addr == named_reg(reg).addr)
                    .count()
            };
            assert_eq!(
                asked(RegId::PresentTemperature),
                1,
                "servo {id} temperature reads"
            );
            // The bounds the fence case judges, on the rows it judges: the sweep
            // reads them, and the fence case reads nothing.
            if leg_index(ROWS[row]).is_some() {
                assert_eq!(asked(RegId::MinPositionLimit), 1, "servo {id} lower");
                assert_eq!(asked(RegId::MaxPositionLimit), 1, "servo {id} upper");
            }
        }
    }

    /// A latched bit beyond input voltage fails; input voltage on its own passes
    /// and is still printed.
    #[test]
    fn only_bits_beyond_input_voltage_fail_health() {
        let cfg = example_config();
        let legs = stow_legs();

        let mut voltage_only = machine_at(&cfg, &legs);
        voltage_only.set(11, named_reg(RegId::HardwareErrorStatus), &[0x01]);
        let (report, _) = run(&cfg, voltage_only);
        assert_eq!(report.outcome(Case::Health), Outcome::Pass);
        assert!(report.to_string().contains("0x01"), "the byte is reported");

        let mut overload = machine_at(&cfg, &legs);
        overload.set(11, named_reg(RegId::HardwareErrorStatus), &[0x20]);
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // The right antenna answering as one of the legs.
        machine.set(
            17,
            named_reg(RegId::ModelNumber),
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(13, named_reg(RegId::HomingOffset), &0i32.to_le_bytes());
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

    /// An antenna resting outside the fold window fails the fold case, naming
    /// that servo and its reading.
    ///
    /// 8250 counts is the 545° one antenna reported immediately after a hard
    /// power cycle on the bench — the one observation on record that says the
    /// fold did not happen, and the reason this case exists. A run that meets it
    /// again fails here rather than planning a sweep in a count frame a turn and
    /// a half from the one the antenna is physically in.
    #[test]
    fn an_antenna_outside_its_turn_fails_the_fold_case_by_name() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            17,
            named_reg(RegId::PresentPosition),
            &8250i32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::AntennaFold), Outcome::Fail);
        let printed = report.to_string();
        assert!(printed.contains("servo 17: 8250 counts"), "{printed}");
        assert!(printed.contains("545.098 deg"), "{printed}");
        // The other antenna is reported and not accused: a fold is per servo.
        assert!(printed.contains("18: 2048 counts"), "{printed}");
        assert!(!printed.contains("servo 18:"), "{printed}");
        assert!(
            report.into_record(1).every_case_passed().is_err(),
            "a machine whose count frame is unknown arms nothing"
        );
    }

    /// The fold case judges the two extended-position joints and nothing else.
    ///
    /// The seven others are single-turn joints whose registers cannot leave one
    /// revolution, so a count outside it there is a reading to diagnose
    /// elsewhere and not a fold that failed. Asserted with a leg parked at a
    /// count no fold would leave, which the clearance case below it does object
    /// to.
    #[test]
    fn the_fold_case_judges_only_the_antennas() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            13,
            named_reg(RegId::PresentPosition),
            &9000i32.to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::AntennaFold), Outcome::Pass);
        let printed = report.to_string();
        let line = printed
            .lines()
            .find(|line| line.starts_with(Case::AntennaFold.slug()))
            .expect("the fold case printed a line");
        assert!(line.contains("17:") && line.contains("18:"), "{line}");
        assert!(!line.contains("13:"), "{line}");
        assert_ne!(
            report.outcome(Case::RestMargins),
            Outcome::Pass,
            "a leg nine thousand counts out is still a machine nobody should arm"
        );
    }

    /// The window's two edges are where the case turns over, counted exactly.
    ///
    /// The whole case is a boundary — a fold leaves a count in the turn, a
    /// wind-down leaves one within half a turn of it, and anything further out
    /// says the fold did not happen — so an off-by-one at either end is the
    /// case admitting the reading it exists to catch. The turn's own edges are
    /// inside it, and the count a session's wind-down actually left on servo 18
    /// (−64) with it.
    #[test]
    fn the_fold_case_turns_over_at_the_edges_of_the_window() {
        for (count, expected) in [
            (0, Outcome::Pass),
            (-64, Outcome::Pass),
            (COUNTS_PER_REV - 1, Outcome::Pass),
            (COUNTS_PER_REV, Outcome::Pass),
            (FOLD_WINDOW.start, Outcome::Pass),
            (FOLD_WINDOW.start - 1, Outcome::Fail),
            (FOLD_WINDOW.end - 1, Outcome::Pass),
            (FOLD_WINDOW.end, Outcome::Fail),
        ] {
            let cfg = example_config();
            let mut machine = machine_at(&cfg, &stow_legs());
            machine.set(17, named_reg(RegId::PresentPosition), &count.to_le_bytes());
            let (report, _) = run(&cfg, machine);

            assert_eq!(
                report.outcome(Case::AntennaFold),
                expected,
                "{count} counts against the fold window"
            );
        }
    }

    /// Both antennas unfolded are both named, and the fold case is one line —
    /// so the second one is not lost behind the first.
    #[test]
    fn two_unfolded_antennas_are_both_named_on_one_line() {
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine.set(
            17,
            named_reg(RegId::PresentPosition),
            &9000i32.to_le_bytes(),
        );
        machine.set(
            18,
            named_reg(RegId::PresentPosition),
            &(-3000i32).to_le_bytes(),
        );
        let (report, _) = run(&cfg, machine);

        assert_eq!(report.outcome(Case::AntennaFold), Outcome::Fail);
        let printed = report.to_string();
        let line = printed
            .lines()
            .find(|line| line.starts_with(Case::AntennaFold.slug()))
            .expect("the fold case printed a line");
        assert!(line.contains("servo 17: 9000 counts"), "{line}");
        assert!(line.contains("servo 18: -3000 counts"), "{line}");
    }

    /// The clearance case reads the resting counts under the bare conversion and
    /// reports what it measured, whatever the verdict on it.
    #[test]
    fn the_clearance_is_measured_off_the_resting_counts() {
        let cfg = example_config();
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        // A servo that stores its goal, holding one a long way from where it
        // stands: 200 counts is about 17.6°, far outside the 2° gate.
        machine.unmirrored = vec![14];
        let resting = i32::from_le_bytes(
            machine
                .get(14, named_reg(RegId::PresentPosition))
                .expect("the fixture rests somewhere")
                .try_into()
                .expect("a position is four bytes"),
        );
        machine.set(
            14,
            named_reg(RegId::GoalPosition),
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        let resting = i32::from_le_bytes(
            machine
                .get(14, named_reg(RegId::PresentPosition))
                .expect("the fixture rests somewhere")
                .try_into()
                .expect("a position is four bytes"),
        );
        machine.set(14, named_reg(RegId::TorqueEnable), &[1]);
        machine.set(
            14,
            named_reg(RegId::GoalPosition),
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
        let cfg = example_config();
        let machine = machine_at(&cfg, &stow_legs());
        let ids = cfg.servo_ids().expect("the roster is nine servos");
        let (report, _, reads) = run_watched(&cfg, machine);

        assert_eq!(report.outcome(Case::GoalShadow), Outcome::Pass);
        let printed = report.to_string();
        for id in ids {
            let asked = |reg: RegId| {
                reads
                    .iter()
                    .filter(|(servo, addr)| *servo == id && *addr == named_reg(reg).addr)
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
    /// at all, and writes that pass into the record a person reviews. Nothing
    /// upstream of the registry judges the figure, so the gate is run where it
    /// is consumed.
    #[test]
    fn a_shadow_tolerance_that_is_not_an_angle_refuses_the_registry() {
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY] {
            let mut cfg = example_config();
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
        let cfg = example_config();
        let mut machine = machine_at(&cfg, &stow_legs());
        machine
            .errors
            .insert((14, named_reg(RegId::PresentPosition).addr), 7);
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
        for (row, joint) in ROWS.into_iter().enumerate() {
            let offset = VENDOR_HOMING_OFFSETS[row];
            match leg_index(joint) {
                Some(leg) => {
                    assert_eq!(offset.abs(), quarter_turn, "leg {}", leg + 1);
                    assert_eq!(
                        offset > 0,
                        leg.is_multiple_of(2),
                        "leg {} takes the other sign",
                        leg + 1
                    );
                }
                None => assert_eq!(offset, 0, "{} is a single-turn joint", Name(joint)),
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
        report.set_rest_counts([2048; ROW_COUNT]);
        report.set_models([1200, 1200, 1200, 1200, 1200, 1200, 1200, 1020, 1020]);
        report.set_datum(DatumRecord::new(
            DatumSetting::Direct,
            VENDOR_HOMING_OFFSETS,
        ));
        report.into_record(1_754_000_000)
    }

    /// The four places are places in the measured set, not interpolations
    /// between them: every figure printed is a span some exchange took.
    #[test]
    fn a_distribution_names_four_spans_that_were_measured() {
        let spans: Vec<Duration> = (1..=100).map(Duration::from_millis).collect();
        let stats = SpanStats::of(&spans);
        assert_eq!(stats.min, Duration::from_millis(1));
        assert_eq!(stats.median, Duration::from_millis(50));
        assert_eq!(stats.p99, Duration::from_millis(99));
        assert_eq!(stats.max, Duration::from_millis(100));
        assert_eq!(format!("{stats}"), "1.000/50.000/99.000/100.000");

        let one = SpanStats::of(&[Duration::from_micros(1500)]);
        assert_eq!(one.min, one.max, "one span is all four places");
        assert_eq!(one.p99, Duration::from_micros(1500));

        assert_eq!(SpanStats::of(&[]), SpanStats::default());

        // At the count the case actually measures, the nearest rank is the
        // 198th of 200 -- the third-worst, which is what its comment says and
        // what a person reads the measurement off.
        let at_the_count: Vec<Duration> = (1..=TIMING_EXCHANGES)
            .map(|ms| Duration::from_millis(u64::try_from(ms).expect("a small count")))
            .collect();
        let two_hundred = SpanStats::of(&at_the_count);
        assert_eq!(two_hundred.p99, Duration::from_millis(198));
        assert_eq!(two_hundred.max, Duration::from_millis(200));
    }

    /// The verdict is what the case exists for: a bus that overran anything, or
    /// a grouped read that came back short, fails and says how many of each.
    #[test]
    fn the_timing_verdict_fails_on_an_overrun_and_on_a_short_grouped_read() {
        let bound = Duration::from_millis(3);
        let inside = ExchangeSpans {
            send: Duration::from_millis(1),
            wait: Duration::from_millis(1),
        };
        let past = ExchangeSpans {
            send: Duration::from_millis(8),
            wait: Duration::from_millis(1),
        };

        let mut fits = TimingRun::new("unicast read", bound);
        fits.note(inside);
        let mut also_fits = TimingRun::new("grouped read", bound);
        also_fits.note(inside);
        let clean = timing_verdict(&fits, &also_fits, 0);
        assert_eq!(clean.outcome, Outcome::Pass);
        assert!(clean.detail.contains("unicast read x1"), "{clean:?}");
        assert!(clean.detail.contains("grouped read x1"), "{clean:?}");

        let mut overran = TimingRun::new("unicast read", bound);
        overran.note(past);
        overran.note(inside);
        let slow = timing_verdict(&overran, &also_fits, 0);
        assert_eq!(slow.outcome, Outcome::Fail);
        assert!(
            slow.detail.starts_with("1 of 3 exchanges ran past"),
            "{slow:?}"
        );
        assert!(
            slow.detail.contains("write 1.000/1.000/8.000/8.000"),
            "the distribution is printed on the failing side too: {slow:?}"
        );

        let short = timing_verdict(&fits, &also_fits, 4);
        assert_eq!(short.outcome, Outcome::Fail);
        assert!(
            short.detail.contains("4 grouped reads came back short"),
            "{short:?}"
        );
    }

    /// The assertion is a count over the exchanges, not a verdict on the worst
    /// of them: how many overran is what says whether a bus stalls now and then
    /// or is simply this slow.
    #[test]
    fn a_timing_run_counts_every_exchange_that_ran_past_its_budget() {
        let bound = Duration::from_millis(3);
        let mut run = TimingRun::new("unicast read", bound);
        for (send, wait) in [(1, 1), (2, 2), (1, 1), (3, 1)] {
            run.note(ExchangeSpans {
                send: Duration::from_millis(send),
                wait: Duration::from_millis(wait),
            });
        }
        assert_eq!(run.len(), 4);
        assert_eq!(run.overruns(), 2, "4 ms and 4 ms are past a 3 ms budget");

        // The budget itself is not an overrun: an exchange that fits exactly
        // fits.
        let mut exact = TimingRun::new("unicast read", bound);
        exact.note(ExchangeSpans {
            send: bound,
            wait: Duration::ZERO,
        });
        assert_eq!(exact.overruns(), 0);

        let printed = format!("{run}");
        assert!(
            printed.starts_with("unicast read x4 against 3.000 ms"),
            "{printed}"
        );
        assert!(
            printed.contains("write 1.000/1.000/3.000/3.000"),
            "{printed}"
        );
        assert!(
            printed.contains("wait 1.000/1.000/2.000/2.000"),
            "{printed}"
        );
        assert!(
            printed.contains("total 2.000/2.000/4.000/4.000"),
            "{printed}"
        );
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
