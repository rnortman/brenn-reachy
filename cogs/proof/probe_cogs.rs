//! The bring-up probe's cog body.
//!
//! This is the whole of what an author writes to run a cog in Rust: one
//! function per cog declared in `probe.clk`, named `execute_<cog name in snake
//! case>` and taking `&mut <CogName>Dial`. The dial type, the `extern "C"`
//! entry point that calls it, and the C++ shim that calls that entry point are
//! all generated from the same `.clk`.
//!
//! What the body does is deliberately thin -- one `GoalSetpoint` datagram per
//! command -- because the point of this cog is the mechanisms around it, not
//! the arithmetic in it. Three of them are load-bearing for the motion cogs and
//! are exercised on every execution:
//!
//! * A `VarPacket` output is filled with `reachy-wire` bytes and a `VarPacket`
//!   input is read back as a slice. That is the parse path every wire-format
//!   channel in the motion system uses, and it is the same whether the channel
//!   is fed by this process or by a socket.
//! * `own_packet` is this cog's own output channel. The sequence number the
//!   next datagram carries is read back out of the last one published rather
//!   than kept in a state field -- a channel is not a queue, so the view over
//!   the ring is a loss-free record of what this cog sent.
//! * An empty `own_packet` view means "never published", which is exactly the
//!   first execution and needs no separate flag.

use brenn_reachy__cogs__proof__probe_clk_rs::ProbeDial;
use clockwork_rs::Clear as _;
use probe_scenario::PROBE_ROW;
use reachy_wire::{GoalSetpoint, JOINT_COUNT, peek_header};

/// Turn each new command into one datagram, and read back the last one sent.
///
/// Every command in the window is counted, but only the last of them reaches
/// the output: an output is a single reserved slot per execution, and an older
/// setpoint superseded within the same execution is one the receiver would have
/// held through anyway. Under the deterministic runner the window never holds
/// more than one message, so the rule is only ever exercised online.
pub fn execute_probe(dial: &mut ProbeDial<'_>) {
    // The header of the last datagram this cog published, or `None` before it
    // ever published one. A malformed packet is not a case: this cog is the
    // only publisher on that channel and it writes whole datagrams, so a decode
    // failure reads as nothing published rather than as damage to recover from.
    let published_seq = dial
        .inputs
        .own_packet
        .latest()
        .and_then(|packet| peek_header(packet.bytes().as_slice()).ok())
        .map(|header| header.seq);

    let mut commands_seen = dial.states.kept.commands_seen();
    let mut latest = None;
    for cmd in dial.inputs.cmds.new_msgs() {
        commands_seen += 1;
        latest = Some((cmd.at().as_nanos(), cmd.position()));
    }
    dial.states.kept.set_commands_seen(commands_seen);
    dial.states.kept.set_echoed_seq(published_seq.unwrap_or(0));

    let Some((execute_at_ns, position)) = latest else {
        return;
    };

    let mut targets = [0.0; JOINT_COUNT];
    targets[PROBE_ROW] = position;
    let goal = GoalSetpoint {
        execute_at_ns,
        mask: 1 << PROBE_ROW,
        targets,
    };
    // The first datagram is sequence 0; every one after is the last one's
    // sequence plus one, wrapping as the wire format says it does.
    let seq = published_seq.map_or(0, |seq| seq.wrapping_add(1));
    let datagram = goal.encode(seq);

    let out = &mut dial.outputs.packet;
    out.msg_mut().clear();
    // The carrier is larger than any datagram this repo defines, so the write
    // cannot be refused for capacity; a false here would mean the two sizes had
    // been changed apart, which is a build-time mistake rather than a runtime
    // case.
    assert!(
        out.msg_mut().try_set_bytes(&datagram),
        "the packet carrier is too small for a GoalSetpoint datagram",
    );
    out.mark_for_publish();
}
