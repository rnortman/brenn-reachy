//! The numbers one crate screens against, beside the numbers they stand in for.
//!
//! The intent edge refuses what the session would refuse, so that a bad sidecar
//! or an over-long timeline is named where an operator can fix it instead of
//! arriving as the session blaming the sender. That only works while the edge's
//! copies of three bounds match the ones they anticipate, and neither side can see
//! the other: `reachy-edge` parses text and links no clip library, and the
//! session's span cap is a number in a deployed textproto. This is the join.
//!
//! The wire contract's name bound, speed pair and reserved pose name are
//! restated across the same kind of gap — `motion-proto` takes no dependency,
//! and neither the asset crates nor `reachy-motion` depend on it — and this is
//! the only place in the tree that links them, so those copies are joined here
//! too. The name bound's mirror is `reachy-motion`'s, which is where the one
//! rule every asset crate shares lives; the speed pair's is `reachy-clips`'s,
//! speed being a clip's; the two reserved base spellings' and the pace
//! ceiling's are `reachy-poses`'s, which is what the document loader holds an
//! authored pose to.

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

#[test]
fn the_edge_screens_a_pose_index_against_the_library_the_box_loads() {
    assert_eq!(
        reachy_edge::MAX_POSES,
        reachy_poses::config::MAX_POSES,
        "the sidecar's pose indices are screened against this number; an index the library \
         does not reach is refused by the mover as a command, which cannot name the sidecar",
    );
}

#[test]
fn the_wire_admits_exactly_the_names_the_asset_format_admits() {
    assert_eq!(
        motion_proto::MAX_ASSET_NAME_LEN,
        reachy_motion::asset_name::MAX_ASSET_NAME_LEN,
        "a name the wire carries and the library refuses is an asset the importer will not \
         take; a name the library holds and the wire refuses is an asset nothing can invoke",
    );
}

#[test]
fn the_wire_admits_exactly_the_speeds_the_asset_format_admits() {
    assert_eq!(
        (motion_proto::MIN_SPEED, motion_proto::MAX_SPEED),
        (
            reachy_clips::format::MIN_SPEED,
            reachy_clips::format::MAX_SPEED
        ),
        "a speed the wire accepts and the library refuses is a script refused at the session, \
         which cannot name the library's bound",
    );
}

#[test]
fn the_library_reserves_the_name_the_wire_reserves() {
    assert_eq!(
        motion_proto::STOW_POSE,
        reachy_poses::STOW_POSE,
        "the wire closes a script with one name and the emitter reserves another: every \
         schedule would end at a pose no library holds",
    );
}

#[test]
fn the_library_refuses_the_name_the_wire_spends_on_holding_still() {
    assert_eq!(
        motion_proto::KEEP_BASE,
        reachy_poses::KEEP_BASE,
        "the wire spends one spelling on holding the base still and the loader reserves \
         another: a pose could be authored under a name no script can ever reach",
    );
}

#[test]
fn the_library_paces_a_move_no_longer_than_the_wire_does() {
    assert_eq!(
        u64::from(reachy_poses::format::MAX_DURATION_MS),
        motion_proto::MAX_TIMEOUT_MS,
        "a pace the library states and the wire refuses is a move the machine plans for \
         itself that no publisher could have asked for",
    );
}
