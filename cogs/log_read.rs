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
//!
//! The pass itself is here too: [`read_with`] opens the log, checks every
//! binding in a consumer's table before decoding anything, dispatches each
//! message to the stream its channel names, and keeps the census of what every
//! channel carried. A consumer declares only its own streams and its own table.

use std::path::Path;

use clockwork_logs::offboard::OffboardReader;
use clockwork_logs::{ChannelMetadata, LogError, LoggedMessage};
use clockwork_rs::{Blob, SchemaMeta};

/// Every channel the log carries, in the order the reader reports them, and how
/// many messages each held.
///
/// A consumer with no Rust type bound to a channel can still say whether
/// anything travelled on it, which is the difference between a report group that
/// reported and one that only exists.
pub type Census = Vec<(String, usize)>;

/// One channel of a system, and the two things done with it: the binding check
/// before anything is decoded, and the decode into the stream it belongs to.
///
/// The same row drives both, so a channel cannot be checked and left unread, or
/// read and left unchecked.
pub struct Bound<R> {
    /// The channel's name in the log, which is the name the system declares.
    pub name: &'static str,
    /// Assert the log records the schema this build is about to decode.
    pub check: fn(&[ChannelMetadata], &str, &mut Complaints),
    /// Decode one message of it into the stream it belongs to.
    pub route: fn(&mut R, &LoggedMessage<'_>),
}

/// What [`read_with`] fills in besides a consumer's own streams.
///
/// A consumer holds the census and the complaints itself rather than receiving
/// them alongside, because everything a checker or a report says about a run is
/// said about one value.
pub trait Streams: Default {
    /// Where the channel census goes.
    fn census(&mut self) -> &mut Census;
    /// Where a decode nobody could make sense of goes.
    fn complaints(&mut self) -> &mut Complaints;
}

/// Read the log under `dir` into `R`, one pass, through `table`.
///
/// The order is the safety order for a reader: the census is opened over every
/// channel the log declares, every binding in the table is checked before a
/// single payload is decoded, and only then are the messages walked. The
/// reader's own error counters are a complaint at the end -- a log the framework
/// had trouble with is not a log anything should be asserted about.
///
/// # Errors
///
/// Whatever the reader refuses about the log as a whole. A message the reader
/// yielded and this build could not make sense of is a complaint rather than an
/// error: the point is to report all of them at once.
pub fn read_with<R: Streams>(dir: &Path, table: &[Bound<R>]) -> Result<R, LogError> {
    let mut reader = OffboardReader::open(dir)?;
    let mut run = R::default();

    let channels: Vec<ChannelMetadata> = reader.channels().to_vec();
    for metadata in &channels {
        run.census().push((metadata.channel_name.clone(), 0));
    }
    for bound in table {
        (bound.check)(&channels, bound.name, run.complaints());
    }

    while let Some(message) = reader.read_next()? {
        let name = message.metadata.channel_name.as_str();
        if let Some(entry) = run.census().iter_mut().find(|(known, _)| known == name) {
            entry.1 += 1;
        }
        if let Some(bound) = table.iter().find(|bound| bound.name == name) {
            (bound.route)(&mut run, &message);
        }
    }

    if !reader.error_counters().is_clean() {
        let counters = format!("{:?}", reader.error_counters());
        run.complaints().push(format!(
            "the reader recorded errors over the log: {counters}"
        ));
    }
    Ok(run)
}

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
