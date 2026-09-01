//! The shape every log analyzer in this tree reports in, and the printer for it.
//!
//! Two analyzers read the two halves of one fetch — the channel log and the
//! voice host's console — and an operator reads their output side by side after
//! the same `make …-fetch`. That output is a contract with a person, not an
//! internal detail, so it is one implementation rather than one per tool: a
//! second copy drifts on the first change nobody made twice, and the divergence
//! is invisible until somebody compares two runs.
//!
//! What a tool keeps for itself is the analysis. What it gets from here is the
//! two lists, the words for adding to them, and the printer that turns them into
//! stdout, stderr and an exit status.

#![forbid(unsafe_code)]

use std::process::ExitCode;

/// What an analyzer concluded about one run.
///
/// Two lists rather than one, because they are read for different reasons. A
/// finding is a claim about the run that did not hold and is what the exit
/// status is about; a measurement is a number the run produced, printed whether
/// the run passed or not. A run that fails is exactly the run whose numbers
/// somebody needs.
#[derive(Default)]
pub struct Report {
    /// The ways the run did not do what it claims to have done.
    pub findings: Vec<String>,
    /// What the run did, for a person to read.
    pub measured: Vec<String>,
}

impl Report {
    /// One way the run did not do what it claims to have done.
    pub fn fail(&mut self, what: impl Into<String>) {
        self.findings.push(what.into());
    }

    /// One thing the run did.
    pub fn note(&mut self, what: impl Into<String>) {
        self.measured.push(what.into());
    }
}

/// Print both halves and answer with the verdict.
///
/// The measurements go to stdout and the findings to stderr: the numbers file
/// with the run record and the findings are what an operator sees on the
/// terminal. `clean` is the one-line sentence a run with no findings gets — the
/// only part of this that is the tool's own.
pub fn verdict(tool: &str, over: &str, report: &Report, clean: &str) -> ExitCode {
    if write_verdict(
        &mut std::io::stdout().lock(),
        &mut std::io::stderr().lock(),
        tool,
        over,
        report,
        clean,
    ) {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// Both halves onto the two sinks, and whether the run held.
///
/// The split is the contract: an operator reads the findings off a terminal
/// while the measurements are redirected into the file that is filed with the
/// run record, and a finding that went to stdout is one nobody was told about.
/// Written to handed-in sinks so a case can hold the two apart, which a printer
/// asserted only through its exit status cannot.
///
/// # Panics
///
/// If either sink cannot be written, which for the caller's own streams is a
/// terminal or a file that went away mid-report.
pub fn write_verdict(
    out: &mut impl std::io::Write,
    err: &mut impl std::io::Write,
    tool: &str,
    over: &str,
    report: &Report,
    clean: &str,
) -> bool {
    writeln!(out, "{tool} over {over}").expect("a writable stream");
    for line in &report.measured {
        writeln!(out, "{line}").expect("a writable stream");
    }
    if report.findings.is_empty() {
        writeln!(out, "{clean}").expect("a writable stream");
        return true;
    }
    for finding in &report.findings {
        writeln!(err, "{tool}: {finding}").expect("a writable stream");
    }
    writeln!(
        err,
        "{tool}: {} finding(s) over {over}",
        report.findings.len()
    )
    .expect("a writable stream");
    false
}

#[cfg(test)]
mod tests {
    use super::{Report, verdict, write_verdict};

    /// What the printer sent to each sink, as text.
    fn printed(report: &Report) -> (bool, String, String) {
        let mut out: Vec<u8> = Vec::new();
        let mut err: Vec<u8> = Vec::new();
        let held = write_verdict(
            &mut out,
            &mut err,
            "tool",
            "somewhere",
            report,
            "it happened",
        );
        (
            held,
            String::from_utf8(out).expect("text"),
            String::from_utf8(err).expect("text"),
        )
    }

    #[test]
    fn a_report_starts_with_nothing_in_either_half() {
        let report = Report::default();
        assert!(report.findings.is_empty());
        assert!(report.measured.is_empty());
    }

    #[test]
    fn each_half_keeps_what_it_was_given_in_order() {
        let mut report = Report::default();
        report.note("first");
        report.fail("broken");
        report.note("second");
        assert_eq!(report.measured, vec!["first", "second"]);
        assert_eq!(report.findings, vec!["broken"]);
    }

    #[test]
    fn a_report_with_no_findings_succeeds() {
        let mut report = Report::default();
        report.note("a number");
        assert_eq!(
            format!("{:?}", verdict("tool", "somewhere", &report, "it happened")),
            format!("{:?}", std::process::ExitCode::SUCCESS)
        );
    }

    #[test]
    fn a_clean_report_says_it_on_stdout_and_says_nothing_on_stderr() {
        let mut report = Report::default();
        report.note("a number");
        let (held, out, err) = printed(&report);
        assert!(held);
        assert_eq!(out, "tool over somewhere\na number\nit happened\n");
        assert_eq!(err, "");
    }

    #[test]
    fn the_findings_go_to_stderr_named_and_counted_and_the_numbers_stay_on_stdout() {
        let mut report = Report::default();
        report.note("a number");
        report.fail("it did not");
        report.fail("nor that");
        let (held, out, err) = printed(&report);
        assert!(!held);
        assert_eq!(out, "tool over somewhere\na number\n");
        assert_eq!(
            err,
            "tool: it did not\ntool: nor that\ntool: 2 finding(s) over somewhere\n"
        );
    }

    #[test]
    fn a_report_with_a_finding_fails() {
        let mut report = Report::default();
        report.fail("it did not");
        assert_eq!(
            format!("{:?}", verdict("tool", "somewhere", &report, "it happened")),
            format!("{:?}", std::process::ExitCode::FAILURE)
        );
    }
}
