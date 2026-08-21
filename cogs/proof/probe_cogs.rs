//! The bring-up probe's cog body.
//!
//! This is the whole of what an author writes to run a cog in Rust: one
//! function per cog declared in `probe.clk`, named `execute_<cog name in snake
//! case>` and taking `&mut <CogName>Dial`. The dial type, the `extern "C"`
//! entry point that calls it, and the C++ shim that calls that entry point are
//! all generated from the same `.clk`.
//!
//! What the body does is deliberately thin -- one setpoint per command --
//! because the point of this cog is the mechanisms around it, not the
//! arithmetic in it. Three of them are load-bearing for the motion cogs and
//! are exercised on every execution:
//!
//! * A `VarPacket` output is filled with a schema's own blob bytes and a
//!   `VarPacket` input is read back as a slice. That is the datagram path a
//!   socket carries: the payload is the bytes of the message both ends declare,
//!   so what crosses is the schema and not a hand-packed copy of it.
//! * `own_packet` is this cog's own output channel. What the last datagram
//!   named is read back out of it rather than kept in a state field -- a
//!   channel is not a queue, so the view over the ring is a loss-free record of
//!   what this cog sent.
//! * An empty `own_packet` view means "never published", which is exactly the
//!   first execution and needs no separate flag.
//! * The state slot is read and written through its validated view: one
//!   `validate_mut` at the top of the body, then plain field assignment and a
//!   real Rust enum, on the slot's own bytes. That is how every cog in this repo
//!   reaches its state -- including the refusal arm, which counts the slot it
//!   had to clear rather than dropping the fact on the floor.

use brenn_reachy__cogs__proof__probe_clk_rs::ProbeDial;
use brenn_reachy__cogs__proof__probe_msgs_clk_rs::ProbeStep;
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use clockwork_rs::{Clear as _, SyncTime, blob_as_bytes, blob_from_bytes};
use probe_scenario::PROBE_MASK;

/// Turn each new command into one datagram, and read back the last one sent.
///
/// Every command in the window is counted, but only the last of them reaches
/// the output: an output is a single reserved slot per execution, and an older
/// setpoint superseded within the same execution is one the receiver would have
/// held through anyway. Under the deterministic runner the window never holds
/// more than one message, so the rule is only ever exercised online.
pub fn execute_probe(dial: &mut ProbeDial<'_>) {
    // The instant the last datagram this cog published named, or `None` before
    // it ever published one. A payload of the wrong size is not a case: this
    // cog is the only publisher on that channel and it writes whole messages,
    // so a refusal reads as nothing published rather than as damage to recover
    // from.
    let published_due = dial
        .inputs
        .own_packet
        .latest()
        .and_then(|packet| blob_from_bytes::<GoalSetpointWire>(packet.bytes().as_slice()))
        .map(|goal| goal.execute_at());

    // The state slot, narrowed once for the whole body. This cog is the only
    // writer and every field of the schema reads back valid from zero, so a
    // refusal here would mean the slot's bytes had been damaged from outside the
    // cog; the body starts from a cleared slot rather than carrying damage
    // forward.
    //
    // The reset is recorded on the slot it clears, because clearing loses every
    // count in it: an unrecorded one reads afterwards exactly like a process
    // that has just started. The counter is read off the open surface first --
    // an integer field has no invalid pattern, so the count survives whatever
    // damaged the slot -- and the failure's offset is kept beside it, which with
    // the schema in hand names the field whose bytes were bad.
    let kept = match dial.states.kept.validate_mut() {
        Ok(kept) => kept,
        Err(invalid) => {
            let resets = dial.states.kept.state_resets().saturating_add(1);
            let kept = dial.states.kept.clear_valid();
            kept.state_resets = resets;
            kept.last_reset_offset = u32::try_from(invalid.offset).unwrap_or(u32::MAX);
            kept
        }
    };

    // Set before the window is read and overwritten only by a publish, so the
    // field says what *this* execution did rather than latching the last one
    // that had something to send.
    kept.last_step = ProbeStep::Idle;

    let mut latest = None;
    for cmd in dial.inputs.cmds.new_msgs() {
        kept.commands_seen += 1;
        latest = Some((cmd.at().as_nanos(), cmd.position()));
    }
    kept.echoed_due = published_due.unwrap_or(SyncTime::from_nanos(0));

    let Some((execute_at_ns, position)) = latest else {
        return;
    };
    kept.last_step = ProbeStep::Published;

    let mut goal = GoalSetpointWire::new();
    goal.set_execute_at(SyncTime::from_nanos(execute_at_ns));
    goal.set_mask(JointFlagsWire::from(PROBE_MASK));
    goal.targets_mut().set_body_yaw(position);

    let out = &mut dial.outputs.packet;
    out.msg_mut().clear();
    // The carrier is larger than any message this repo sends this way, so the
    // write cannot be refused for capacity; a false here would mean the two
    // sizes had been changed apart, which is a build-time mistake rather than a
    // runtime case.
    assert!(
        out.msg_mut().try_set_bytes(blob_as_bytes(&goal)),
        "the packet carrier is too small for a setpoint",
    );
    out.mark_for_publish();
}
