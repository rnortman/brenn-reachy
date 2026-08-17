//! Taking a Clockwork output log apart, for whatever system wrote it.
//!
//! Every scenario in this repo ends the same way: open the log, satisfy
//! yourself that each channel carries the schema you are about to decode it as,
//! and lift the messages out of the reader's borrowed view so the whole run can
//! be looked at before anything is asserted. None of that is about motion, or
//! about any particular system, so it is here rather than copied into each
//! system's reader -- including the harness proof's, which exists to catch a
//! framework break before a motion scenario meets it and must therefore keep
//! working when the motion schemas change.
//!
//! Nothing here asserts anything about a run. What it does refuse is a decode
//! nobody checked: [`binding`] compares the log's own schema name and definition
//! against the type, because a payload that happens to be the right size is not
//! the right message, and a checker that decoded one as the other would report
//! nonsense about a machine.
//!
//! Every complaint is collected rather than thrown. A checker that stopped at
//! the first surprise costs a whole build per finding.

use clockwork_logs::{ChannelMetadata, LoggedMessage};
use clockwork_rs::{Blob, SchemaMeta};

use clockwork__clockwork__io__var_packet_clk_rs::{VarPacket__64, VarPacket__128, VarPacket__288};
use reachy_wire::{DecodeError, Header};

/// What went wrong reading a log: a channel bound to a schema this build does
/// not know, a payload that did not decode. Every one of these is a failure of
/// the run.
pub type Complaints = Vec<String>;

/// One message as a checker sees it: what it says, and when the log says it was
/// sent.
pub struct Logged<T> {
    /// The instant the message was published, which for anything a cog wrote is
    /// its execution's start time plus the modelled execution duration -- not
    /// the instant the message's contents are about.
    pub at_ns: i64,
    /// The publisher's own count of what it has sent on this channel.
    pub sequence_number: u32,
    /// The message.
    pub message: T,
}

/// A datagram, as both bytes and the value they decode to.
///
/// The bytes are kept because the carrier is meant to be transparent: a scenario
/// that wants to say "exactly what the cog encoded came back" compares them, and
/// one that wants to reason about the run reads the value.
pub struct Datagram<T> {
    /// The wire header's sequence number, which is the sender's count and not
    /// the channel publisher's.
    pub wire_seq: u32,
    /// The datagram as it travelled.
    pub bytes: Vec<u8>,
    /// What it says.
    pub message: T,
}

/// A schema type that carries opaque bytes: one of `var_packet.clk`'s own
/// instantiations, since a repo may not instantiate a generic another module
/// declares.
///
/// The trait exists so one decode helper serves all three sizes; a channel's
/// size is a property of the channel and not of what rides on it.
pub trait Carrier {
    /// The datagram inside, copied out of the carrier's borrowed view.
    fn payload(&self) -> Vec<u8>;
}

macro_rules! carrier {
    ($($type:ty),+ $(,)?) => {
        $(
            impl Carrier for $type {
                fn payload(&self) -> Vec<u8> {
                    self.bytes().as_slice().to_vec()
                }
            }
        )+
    };
}

carrier!(VarPacket__64, VarPacket__128, VarPacket__288);

/// One complaint if `name` is missing from the log, or carries a schema other
/// than the one about to be decoded from it.
///
/// Called for every bound channel before a single message is read, including the
/// channels a run expects to be empty: zero messages on a channel means silence
/// only if the channel is there and carries what it claims to.
pub fn binding<T: SchemaMeta>(
    channels: &[ChannelMetadata],
    name: &str,
    complaints: &mut Complaints,
) {
    match channels.iter().find(|channel| channel.channel_name == name) {
        None => complaints.push(format!("no channel named {name} in the log")),
        Some(metadata) if !metadata.carries::<T>() => complaints.push(format!(
            "{name} carries {:?}, not {}",
            metadata.schema_name,
            T::SCHEMA_NAME
        )),
        Some(_) => {}
    }
}

/// Decode one message as the schema type its channel is bound to, and keep it.
pub fn typed<T: Blob + SchemaMeta>(
    message: &LoggedMessage<'_>,
    out: &mut Vec<Logged<T>>,
    complaints: &mut Complaints,
) {
    match message.to_message::<T>() {
        Ok(decoded) => out.push(Logged {
            at_ns: message.message_time.as_nanos(),
            sequence_number: message.sequence_number,
            message: decoded,
        }),
        Err(err) => complaints.push(complaint(message, T::SCHEMA_NAME, &err)),
    }
}

/// The wire codec's entry point for one message type: the bytes in, the header
/// and the message out, or why they are neither.
pub type Decode<T> = fn(&[u8]) -> Result<(Header, T), DecodeError>;

/// The same for a channel of datagrams: out of the carrier, then through the
/// wire codec.
///
/// `decode` is the codec's own entry point for the message type this channel
/// carries -- the checker decodes with the same code the cog encoded with, so
/// what is under test is the run and never the encoding.
pub fn datagram<C: Blob + SchemaMeta + Carrier, T>(
    message: &LoggedMessage<'_>,
    decode: Decode<T>,
    out: &mut Vec<Logged<Datagram<T>>>,
    complaints: &mut Complaints,
) {
    let carrier = match message.to_message::<C>() {
        Ok(carrier) => carrier,
        Err(err) => {
            complaints.push(complaint(message, C::SCHEMA_NAME, &err));
            return;
        }
    };
    let bytes = carrier.payload();
    match decode(&bytes) {
        Ok((header, decoded)) => out.push(Logged {
            at_ns: message.message_time.as_nanos(),
            sequence_number: message.sequence_number,
            message: Datagram {
                wire_seq: header.seq,
                bytes,
                message: decoded,
            },
        }),
        Err(err) => complaints.push(complaint(
            message,
            core::any::type_name::<T>(),
            &format!("{err}"),
        )),
    }
}

/// One complaint, in the one shape they all take: which channel, which instant,
/// what it was expected to be, and what the refusal said.
///
/// Uniform because these lines are what a failing scenario prints, and three
/// phrasings of the same failure read as three different problems.
fn complaint(message: &LoggedMessage<'_>, wanted: &str, why: &dyn core::fmt::Display) -> String {
    format!(
        "{} at {}: not a {wanted}: {why}",
        message.metadata.channel_name,
        message.message_time.as_nanos(),
    )
}
