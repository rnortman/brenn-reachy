//! The numbers the edge screens against, beside the numbers they stand in for.
//!
//! The intent edge refuses what the session would refuse, so that a bad sidecar
//! or an over-long timeline is named where an operator can fix it instead of
//! arriving as the session blaming the sender. That only works while the edge's
//! copies of two bounds match the ones they anticipate, and neither side can see
//! the other: `reachy-edge` parses text and links no clip library, and the
//! session's span cap is a number in a deployed textproto. This is the join.

use std::path::PathBuf;

const SESSION_PARAMS: &str = "SESSION_PARAMS";

fn span_cap_ms() -> u64 {
    let path = PathBuf::from(std::env::var(SESSION_PARAMS).expect("the runfile's path"));
    let text = std::fs::read_to_string(&path).expect("the deployed session parameters");
    let line = text
        .lines()
        .map(str::trim)
        .find(|line| line.starts_with("script_span_cap_ms:"))
        .expect("the parameters state a span cap");
    line.split(':')
        .nth(1)
        .expect("a field and its value")
        .trim()
        .parse()
        .expect("a span cap in milliseconds")
}

#[test]
fn the_session_admits_every_timeline_the_wire_contract_permits() {
    assert!(
        span_cap_ms() >= motion_proto::MAX_TIMEOUT_MS,
        "a script may state a {} ms timeout and the session caps a span at {} ms: the edge \
         would forward what the session refuses, on the machine, blaming the sender",
        motion_proto::MAX_TIMEOUT_MS,
        span_cap_ms(),
    );
}

#[test]
fn the_edge_screens_a_motion_index_against_the_library_the_box_loads() {
    assert_eq!(
        reachy_edge::MAX_MOTIONS,
        reachy_clips::config::MAX_MOTIONS,
        "the sidecar's indices are screened against this number; an index the library \
         does not reach is refused at the session, which cannot name the sidecar",
    );
}
