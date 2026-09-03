//! The shipped configuration.
//!
//! One claim, and it is about a file rather than about code: the configuration
//! the driver ships with parses — through `load`, the entry point the driver
//! itself calls, rather than a re-reading of the file beside it — and carries
//! the values it is meant to. A file that only the device ever reads is a file
//! nothing checks; this is what makes it checked.
//!
//! Field coverage — that the parser accepts exactly what the schema declares —
//! is the parser's own invariant: it walks the descriptor, so a field the
//! schema declares and the reader does not carry is a refusal on every
//! configuration, this file's first case among them.

use reachy_motord::params;
use std::path::Path;

/// The environment variable naming the shipped configuration, relative to the
/// runfiles root, which is a test's working directory.
const PARAMS_ENV: &str = "MOTORD_PARAMS";

/// What `name` points at.
///
/// Panics rather than answers: a missing runfile is a broken test target, not a
/// case.
fn path_of(name: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| {
        panic!(
            "{name} is unset: the test target has to name the file beside the data attribute that \
             supplies it"
        )
    })
}

/// The contents of the file `name` points at.
///
/// Panics rather than answers: a missing runfile is a broken test target, not a
/// case.
fn runfile(name: &str) -> String {
    let path = path_of(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{name} names {path}, which does not read: {error}"))
}

#[test]
fn the_shipped_configuration_is_the_one_the_driver_runs() {
    let message =
        params::load(Path::new(&path_of(PARAMS_ENV))).expect("the shipped configuration parses");
    let shipped = message.validate().expect("a parsed message validates");

    assert_eq!(
        shipped.period_ns,
        reachy_driver::NOMINAL_CYCLE_NS,
        "the driver's cycle is the grid its own budgets are sized against, not a number beside it"
    );
    assert_eq!(
        shipped.hold_timeout_ns, 200_000_000,
        "the dead-man the simulated driver runs, so a scenario's evidence about it carries"
    );
    assert_eq!(
        shipped.health_poll_period_ns, 120_000_000,
        "the spacing the simulated driver runs, so a lap of the nine is about a second"
    );
    assert_eq!(
        shipped.bus_device.as_str(),
        "/dev/ttyAMA3",
        "the servo bus node on the board"
    );
    assert_eq!(shipped.bus_baud, 1_000_000);
    // The relations between these — a dead-man of whole cycles, longer than one —
    // are the parser's own refusals and are pinned there, over every file rather
    // than over this one.
}

#[test]
fn a_configuration_that_omits_a_field_is_refused_rather_than_defaulted() {
    // The shipped file with its cycle taken out: the one case that must not
    // silently become a zero, because a zero period is a loop that spins.
    let without = runfile(PARAMS_ENV)
        .lines()
        .filter(|line| !line.trim_start().starts_with("period_ns:"))
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(
        params::parse(&without),
        Err(params::ParamsErrorKind::MissingField { name: "period_ns" })
    );
}
