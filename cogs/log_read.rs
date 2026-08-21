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

/// One complaint, in the one shape they all take: which channel, which instant,
/// what it was expected to be, and what the refusal said.
///
/// Uniform because these lines are what a failing scenario prints, and two
/// phrasings of the same failure read as two different problems.
fn complaint(message: &LoggedMessage<'_>, wanted: &str, why: &dyn core::fmt::Display) -> String {
    format!(
        "{} at {}: not a {wanted}: {why}",
        message.metadata.channel_name,
        message.message_time.as_nanos(),
    )
}
