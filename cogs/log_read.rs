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
//! Either of Clockwork's two log formats is read: a deterministic run writes
//! offboard `.slog` files and the online logger writes onboard `.olog` ones, and
//! the same checker reads either. Which is not a detail -- the analyzer a
//! hardware run is judged by would otherwise be proven over a format no machine
//! produces.
//!
//! The pass itself is here too: [`read_with`] opens the log, checks every
//! binding in a consumer's table before decoding anything, dispatches each
//! message to the stream its channel names, and keeps the census of what every
//! channel carried. A consumer declares only its own streams and its own table.

use std::path::Path;

use clockwork_logs::{ChannelMetadata, LogError, LogFormat, LogReader, LoggedMessage};
use clockwork_rs::{Blob, SchemaMeta};

/// Every channel the log carries, in the order the reader reports them.
///
/// A consumer with no Rust type bound to a channel can still say whether
/// anything travelled on it, which is the difference between a report group that
/// reported and one that only exists.
pub type Census = Vec<Channel>;

/// One channel of a log: its name, how much of it the log holds, and where in
/// the publisher's own count the log's copy starts.
pub struct Channel {
    /// The channel's name as the log declares it.
    pub name: String,
    /// How many messages of it the log holds.
    pub count: usize,
    /// The lowest sequence number the log holds on it, or nothing where the log
    /// holds no message of it at all.
    ///
    /// The publisher numbers from zero and never skips, so a channel whose
    /// lowest recorded number is not zero is a channel the log is short at the
    /// front of: the subscriber attached after those publishes and they are
    /// gone. That is a fact about the recording rather than about the run, and
    /// it is the one thing a count on its own cannot say.
    pub first_seq: Option<u32>,
}

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
/// Either format is read. What differs is what a channel's absence means, which
/// is a property of the formats and not of a run: see the comment on the check
/// loop.
///
/// # Errors
///
/// Whatever the reader refuses about the log as a whole. A message the reader
/// yielded and this build could not make sense of is a complaint rather than an
/// error: the point is to report all of them at once.
pub fn read_with<R: Streams>(dir: &Path, table: &[Bound<R>]) -> Result<R, LogError> {
    let (format, channels) = channels_of(dir)?;
    let mut run = R::default();

    for metadata in &channels {
        run.census().push(Channel {
            name: metadata.channel_name.clone(),
            count: 0,
            first_seq: None,
        });
    }
    for bound in table {
        // A channel an onboard log never carried a message on is a channel that
        // log does not declare: the writer declares one when it first writes to
        // it. So absence there says the channel was silent, not that the system
        // was misconfigured, and there is nothing to check -- no payload of it
        // will be decoded. An offboard log declares its whole channel set
        // whatever travelled on it, so absence there is a real finding.
        if format == LogFormat::Onboard
            && !channels
                .iter()
                .any(|metadata| metadata.channel_name == bound.name)
        {
            continue;
        }
        (bound.check)(&channels, bound.name, run.complaints());
    }

    let mut reader = LogReader::open(dir)?;
    while let Some(message) = reader.read_next()? {
        let name = message.metadata.channel_name.as_str();
        if let Some(entry) = run.census().iter_mut().find(|channel| channel.name == name) {
            entry.count += 1;
            // The lowest rather than the first walked: what is wanted is where
            // the log's copy of a channel starts, and the reader walks a log in
            // time order across channels rather than in any one publisher's
            // order.
            entry.first_seq = Some(match entry.first_seq {
                Some(lowest) => lowest.min(message.sequence_number),
                None => message.sequence_number,
            });
        }
        if let Some(bound) = table.iter().find(|bound| bound.name == name) {
            (bound.route)(&mut run, &message);
        }
    }

    let damage =
        match &reader {
            LogReader::Onboard(reader) => (!reader.error_counters().is_clean())
                .then(|| format!("{:?}", reader.error_counters())),
            LogReader::Offboard(reader) => (!reader.error_counters().is_clean())
                .then(|| format!("{:?}", reader.error_counters())),
            // The compiler's arm; `channels_of` has already refused any
            // format that would reach it.
            _ => None,
        };
    if let Some(counters) = damage {
        run.complaints().push(format!(
            "the reader recorded errors over the log: {counters}"
        ));
    }
    Ok(run)
}

/// Which format a log is in, and every channel it declares.
///
/// The two formats say what they carry at different times, and neither answer is
/// the other's. An offboard log declares its channel set in the trailers the
/// reader reads when it opens, so the set is known before a record is. An onboard
/// log declares a channel in a record of its own, by an id local to the file it
/// is in, and the reader's own table of them is about the file it is currently
/// reading -- so the way to know what a whole onboard log carries is to walk it
/// and keep each message's own metadata.
///
/// That walk is a pass of its own, before the caller's. It costs a second read of
/// a file a run of this system writes a megabyte of, and it buys the same safety
/// order for both formats: every binding in a table checked before a single
/// payload is decoded. Nothing here decodes one -- a message's metadata is read
/// and its payload is left alone.
fn channels_of(dir: &Path) -> Result<(LogFormat, Vec<ChannelMetadata>), LogError> {
    match LogReader::open(dir)? {
        LogReader::Offboard(reader) => Ok((LogFormat::Offboard, reader.channels().to_vec())),
        LogReader::Onboard(mut reader) => {
            let mut seen: Vec<ChannelMetadata> = Vec::new();
            while let Some(message) = reader.read_next()? {
                if !seen
                    .iter()
                    .any(|known| known.channel_name == message.metadata.channel_name)
                {
                    seen.push(message.metadata.clone());
                }
            }
            Ok((LogFormat::Onboard, seen))
        }
        // `LogReader` is non-exhaustive, so this arm is the compiler's. A format
        // this build does not know is refused rather than read as an empty
        // onboard log: an empty channel set would skip every binding check and
        // decode payloads unexamined.
        _ => Err(LogError::UnknownLogFormat {
            path: dir.to_path_buf(),
        }),
    }
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
///
/// The identity checked is the whole one: the schema's name *and* its recorded
/// definition, which carries every field and every enum value by name. So a log
/// written before a schema grew a value or a field is refused here, and that is
/// the behaviour rather than an oversight. A schema's numbering is append-only
/// so that a recorded log and a running build agree on what a number means, and
/// that is a different guarantee from a new build reading an old log: appending
/// a field changes the wire size, and payloads written at the old size do not
/// decode at all ([`typed`] refuses them). A log is therefore read by a build of
/// the schemas it was written under -- for the logs already on disk, the build
/// they were recorded with -- and the two refusals here and in [`typed`] are
/// what say so out loud instead of decoding one schema's bytes as another's.
// TODO(olog-schema-evolution): softening this check is never the fix.
pub fn binding<T: SchemaMeta>(
    channels: &[ChannelMetadata],
    name: &str,
    complaints: &mut Complaints,
) {
    match channels.iter().find(|channel| channel.channel_name == name) {
        None => complaints.push(format!("no channel named {name} in the log")),
        Some(metadata) if metadata.schema_name != T::SCHEMA_NAME => complaints.push(format!(
            "{name} carries {:?}, not {}",
            metadata.schema_name,
            T::SCHEMA_NAME
        )),
        // Same name, other identity: the recorded definition or the message
        // encoding differs. Said as its own thing, because the complaint above
        // would print the one name twice and read as nonsense.
        Some(metadata) if !metadata.carries::<T>() => complaints.push(format!(
            "{name} carries a {} recorded under another schema definition or encoding than this \
             build's, so it was written by a build whose schemas differ from these",
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

/// Decode one message as the schema type its channel is bound to, and hand it to
/// `take` without keeping it.
///
/// What [`typed`] is for a consumer that folds instead of collecting: a channel
/// carrying one message per control sample over a session an operator ends when
/// they choose is a channel whose retained copy is unbounded in the run's
/// duration, and a consumer that only needs running maxima off it should never
/// hold the samples at all.
pub fn each<T: Blob + SchemaMeta>(
    message: &LoggedMessage<'_>,
    complaints: &mut Complaints,
    take: impl FnOnce(Logged<T>),
) {
    match message.to_message::<T>() {
        Ok(decoded) => take(Logged {
            at_ns: message.message_time.as_nanos(),
            sequence_number: message.sequence_number,
            message: decoded,
        }),
        Err(err) => complaints.push(complaint(message, T::SCHEMA_NAME, &err)),
    }
}

/// Decode one message as the cumulative record its channel carries, and keep the
/// rows it holds as the whole of that stream.
///
/// A cumulative channel republishes its whole content every time it changes, so
/// the newest message is the account and every earlier one is a prefix of it.
/// Each message therefore replaces the stream rather than extending it, which is
/// what makes the stream immune to a reader that attached late: whichever copy
/// was seen first says everything the ones before it said.
///
/// `rows` is what takes the record apart, because which container holds the rows
/// is the schema's business and not this module's. It is handed the whole
/// decoded record, so a reader that needs a field the record carries beside its
/// rows -- how many of them the publisher dropped, say -- takes it there rather
/// than decoding the message a second time.
pub fn cumulative<T: Blob + SchemaMeta, U>(
    message: &LoggedMessage<'_>,
    out: &mut Vec<Logged<U>>,
    complaints: &mut Complaints,
    rows: impl FnOnce(&T, &mut Vec<U>),
) {
    match message.to_message::<T>() {
        Ok(decoded) => {
            let mut held = Vec::new();
            rows(&decoded, &mut held);
            out.clear();
            out.extend(held.into_iter().map(|row| Logged {
                at_ns: message.message_time.as_nanos(),
                sequence_number: message.sequence_number,
                message: row,
            }));
        }
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

#[cfg(test)]
mod tests {
    //! What the format dispatch decides, over logs written here.
    //!
    //! The rule this exists for is the onboard skip: a channel an onboard log
    //! does not declare is not checked, because that format declares a channel
    //! only when it first carries a message. It is the one place where "no
    //! finding" and "not checked" meet, so a drift in how channel names are
    //! compared would skip every binding in a table and decode nothing while
    //! reporting nothing -- over a hardware log, which is the only kind the
    //! onboard path ever reads. So the skip is asserted in both directions and
    //! against the offboard format, where the same absence is a real finding.
    //!
    //! The logs are written by the framework's own writers into a scratch
    //! directory. Nothing here is a fixture of a log's bytes: a hand-built file
    //! would be this module's idea of the format rather than the format.

    use super::{
        Bound, Census, Complaints, Logged, Streams, binding, cumulative, each, read_with, typed,
    };
    use clockwork_logs::offboard::{OffboardWriter, OffboardWriterConfig};
    use clockwork_logs::onboard::OnboardWriter;
    use clockwork_logs::{ChannelMetadata, LogError, LoggedMessage, MessageEncoding, MessageFlags};
    use clockwork_rs::{Blob, SchemaMeta, SyncTime, blob_as_bytes, blob_type};
    use std::borrow::Cow;
    use std::path::{Path, PathBuf};

    blob_type! {
        /// Stands in for a generated message a channel is bound to.
        struct Sample, size = 8, align = 8
    }

    impl SchemaMeta for Sample {
        const SCHEMA_NAME: &str = "@brenn_reachy::cogs::log_read::Sample";
        const SCHEMA_DEFINITION: &[u8] = b"\x0a\x06Sample";
    }

    blob_type! {
        /// Another schema of the same size, which is what makes a size check no
        /// check at all.
        struct Other, size = 8, align = 8
    }

    impl SchemaMeta for Other {
        const SCHEMA_NAME: &str = "@brenn_reachy::cogs::log_read::Other";
        const SCHEMA_DEFINITION: &[u8] = b"\x0a\x05Other";
    }

    blob_type! {
        /// `Sample` after an append that costs no bytes on the wire: one more
        /// value in an enum one of its fields names. The generator writes every
        /// value's name into the definition, so the identity moves and the
        /// layout does not.
        struct Grown, size = 8, align = 8
    }

    impl SchemaMeta for Grown {
        const SCHEMA_NAME: &str = "@brenn_reachy::cogs::log_read::Sample";
        const SCHEMA_DEFINITION: &[u8] = b"\x0a\x06Sample\x2a\x05added";
    }

    blob_type! {
        /// `Sample` after an append that does cost bytes: one more field. Same
        /// name, longer definition, and eight bytes wider than anything written
        /// before it.
        struct Widened, size = 16, align = 8
    }

    impl SchemaMeta for Widened {
        const SCHEMA_NAME: &str = "@brenn_reachy::cogs::log_read::Sample";
        const SCHEMA_DEFINITION: &[u8] = b"\x0a\x06Sample\x32\x05extra";
    }

    /// The two channels a case binds: one it writes and one it does not.
    const SPOKEN: &str = "spoken";
    const SILENT: &str = "silent";

    /// A consumer of both channels.
    #[derive(Default)]
    struct Read {
        samples: Vec<Logged<Sample>>,
        census: Census,
        complaints: Complaints,
    }

    impl Streams for Read {
        fn census(&mut self) -> &mut Census {
            &mut self.census
        }

        fn complaints(&mut self) -> &mut Complaints {
            &mut self.complaints
        }
    }

    /// The table: both channels bound to `Sample`, both checked and both routed.
    fn table() -> Vec<Bound<Read>> {
        fn check(channels: &[ChannelMetadata], name: &str, complaints: &mut Complaints) {
            binding::<Sample>(channels, name, complaints);
        }
        fn route(run: &mut Read, message: &LoggedMessage<'_>) {
            typed::<Sample>(message, &mut run.samples, &mut run.complaints);
        }
        vec![
            Bound {
                name: SPOKEN,
                check,
                route,
            },
            Bound {
                name: SILENT,
                check,
                route,
            },
        ]
    }

    fn scratch(what: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "log-read-{what}-{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|since| since.as_nanos())
                .unwrap_or_default()
        ));
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    fn payload() -> [u8; Sample::SIZE] {
        [1, 2, 3, 4, 5, 6, 7, 8]
    }

    /// An onboard log carrying one message on `SPOKEN`, under the metadata
    /// `carries` is given.
    fn onboard_log(dir: &Path, metadata: &ChannelMetadata) {
        let mut writer = OnboardWriter::create(dir, "run_").expect("an onboard log");
        let channel = writer.add_channel(metadata).expect("a channel");
        writer
            .log_message(
                channel,
                0,
                SyncTime::from_nanos(1_000),
                SyncTime::from_nanos(1_000),
                &[],
                &payload(),
            )
            .expect("a message");
        writer.close().expect("a closed log");
    }

    #[test]
    fn a_channel_an_onboard_log_never_declared_is_silence_rather_than_a_finding() {
        let dir = scratch("onboard-absent");
        onboard_log(&dir, &ChannelMetadata::for_schema::<Sample>(SPOKEN));

        let run = read_with::<Read>(&dir, &table()).expect("an onboard log reads");

        assert!(
            run.complaints.is_empty(),
            "an onboard log declares a channel when it first carries a message, so a channel \
             it never declared is a channel nothing was published on: {:?}",
            run.complaints
        );
        assert_eq!(
            run.samples.len(),
            1,
            "the channel that did carry was decoded"
        );
        assert_eq!(blob_as_bytes(&run.samples[0].message), payload());
        assert_eq!(
            run.census
                .iter()
                .map(|channel| (channel.name.as_str(), channel.count, channel.first_seq))
                .collect::<Vec<_>>(),
            vec![(SPOKEN, 1, Some(0))],
            "the census names what the log declared, counts what it carried once, and says \
             which of the publisher's numbers the log's copy starts at"
        );
    }

    #[test]
    fn a_channel_an_onboard_log_declares_under_another_schema_is_still_a_finding() {
        let dir = scratch("onboard-mismatch");
        let mut metadata = ChannelMetadata::for_schema::<Other>(SPOKEN);
        metadata.message_encoding = MessageEncoding::Tachyon;
        onboard_log(&dir, &metadata);

        let run = read_with::<Read>(&dir, &table()).expect("an onboard log reads");

        assert_eq!(
            run.complaints.len(),
            1,
            "the declared channel is checked and the undeclared one is not: {:?}",
            run.complaints
        );
        assert!(
            run.complaints[0].contains(SPOKEN) && run.complaints[0].contains("Other"),
            "the complaint names the channel and what it actually carries: {:?}",
            run.complaints
        );
    }

    #[test]
    fn the_same_absence_over_an_offboard_log_is_a_finding() {
        let dir = scratch("offboard-absent");
        let mut writer =
            OffboardWriter::create(&dir, OffboardWriterConfig::default()).expect("an offboard log");
        let channel = writer
            .create_channel_typed::<Sample>(SPOKEN)
            .expect("a channel");
        writer
            .write(
                channel,
                0,
                SyncTime::from_nanos(1_000),
                SyncTime::from_nanos(1_000),
                &[],
                &payload(),
            )
            .expect("a message");
        writer.close().expect("a closed log");

        let run = read_with::<Read>(&dir, &table()).expect("an offboard log reads");

        assert_eq!(
            run.complaints,
            vec![format!("no channel named {SILENT} in the log")],
            "an offboard log declares its whole channel set, so a channel missing from it is a \
             system that did not compose the way the table says"
        );
        assert_eq!(run.samples.len(), 1);
    }

    /// What a schema append does to a log recorded before it, both halves.
    ///
    /// The append-only rule on a `.clk` vocabulary buys agreement about what a
    /// number means, not the ability to read yesterday's log with today's
    /// build. This pins the second part, because it is the part that surprises:
    /// the recorded definition carries every value and field by name, so a
    /// channel written before the append fails the binding check, and a payload
    /// written before a *field* was appended is the wrong size to decode at
    /// all. A log outlives the build that wrote it only in the sense that the
    /// build that wrote it can still read it.
    #[test]
    fn a_schema_that_only_grew_is_still_another_schema_to_a_log_written_before_it() {
        let recorded = vec![ChannelMetadata::for_schema::<Sample>(SPOKEN)];
        let mut complaints = Complaints::new();

        binding::<Grown>(&recorded, SPOKEN, &mut complaints);

        assert_eq!(
            complaints.len(),
            1,
            "an appended enum value moves the recorded definition, and the check is on the whole \
             identity: {complaints:?}"
        );
        assert!(
            complaints[0].contains(SPOKEN) && complaints[0].contains("another schema definition"),
            "the complaint says which channel and that the definition is what differs, rather \
             than printing the one schema name twice: {complaints:?}"
        );
    }

    #[test]
    fn a_payload_written_before_a_field_was_appended_does_not_decode_either() {
        let recorded = ChannelMetadata::for_schema::<Sample>(SPOKEN);
        let payload = payload();
        let message = LoggedMessage {
            channel_id: 1,
            metadata: &recorded,
            sequence_number: 0,
            log_time: SyncTime::from_nanos(1_000),
            message_time: SyncTime::from_nanos(1_000),
            header: &[],
            payload: Cow::Borrowed(&payload),
            flags: MessageFlags::default(),
        };
        let mut kept: Vec<Logged<Widened>> = Vec::new();
        let mut complaints = Complaints::new();

        typed::<Widened>(&message, &mut kept, &mut complaints);

        assert!(kept.is_empty(), "nothing of another size is kept");
        assert_eq!(
            complaints.len(),
            1,
            "a payload of the old width is refused rather than read short: {complaints:?}"
        );
        assert!(
            complaints[0].contains(SPOKEN) && complaints[0].contains("1000"),
            "the complaint names the channel and the instant, like every other one: {complaints:?}"
        );
    }

    /// One logged message over `payload`, as a reader hands it to a route.
    fn logged<'a>(
        metadata: &'a ChannelMetadata,
        sequence_number: u32,
        at_ns: i64,
        payload: &'a [u8],
    ) -> LoggedMessage<'a> {
        LoggedMessage {
            channel_id: 1,
            metadata,
            sequence_number,
            log_time: SyncTime::from_nanos(at_ns),
            message_time: SyncTime::from_nanos(at_ns),
            header: &[],
            payload: Cow::Borrowed(payload),
            flags: MessageFlags::default(),
        }
    }

    /// The rows a `Sample` holds, for a case about routing: one per nonzero
    /// byte, so a copy that says more than the one before it is a superset of
    /// it, which is the shape a cumulative record has.
    fn nonzero(sample: &Sample, rows: &mut Vec<u8>) {
        rows.extend(blob_as_bytes(sample).iter().copied().filter(|b| *b != 0));
    }

    /// Each copy of a cumulative record replaces the stream, and does not extend
    /// it.
    ///
    /// The property the whole retention argument rests on: every copy is the
    /// account so far, so whichever one a late reader saw first says everything
    /// the ones before it said. Extending instead would say every row of every
    /// copy, which over a story republished on each append is every row counted
    /// as many times as the session went on to speak.
    #[test]
    fn each_copy_of_a_cumulative_record_replaces_the_rows_before_it() {
        let recorded = ChannelMetadata::for_schema::<Sample>(SPOKEN);
        let first = [1u8, 2, 0, 0, 0, 0, 0, 0];
        let second = [1u8, 2, 3, 0, 0, 0, 0, 0];
        let mut kept: Vec<Logged<u8>> = Vec::new();
        let mut complaints = Complaints::new();

        cumulative::<Sample, u8>(
            &logged(&recorded, 0, 1_000, &first),
            &mut kept,
            &mut complaints,
            nonzero,
        );
        cumulative::<Sample, u8>(
            &logged(&recorded, 4, 2_000, &second),
            &mut kept,
            &mut complaints,
            nonzero,
        );

        assert!(complaints.is_empty(), "{complaints:?}");
        assert_eq!(
            kept.iter().map(|row| row.message).collect::<Vec<_>>(),
            vec![1, 2, 3],
            "the newest copy's rows, and not the two copies concatenated",
        );
        assert_eq!(
            kept.iter()
                .map(|row| (row.at_ns, row.sequence_number))
                .collect::<Vec<_>>(),
            vec![(2_000, 4); 3],
            "every row is stamped with the copy it was read out of",
        );
    }

    /// A copy that will not decode is a complaint, and leaves the rows alone.
    ///
    /// The alternative is worse than a missing complaint: rows cleared for a
    /// record nothing could read would turn a channel recorded under another
    /// schema revision into an empty story, which reads as a session that never
    /// said anything.
    #[test]
    fn a_cumulative_copy_that_does_not_decode_is_a_complaint_and_keeps_the_rows() {
        let recorded = ChannelMetadata::for_schema::<Sample>(SPOKEN);
        let payload = [1u8, 2, 0, 0, 0, 0, 0, 0];
        let mut kept: Vec<Logged<u8>> = Vec::new();
        let mut complaints = Complaints::new();
        cumulative::<Sample, u8>(
            &logged(&recorded, 0, 1_000, &payload),
            &mut kept,
            &mut complaints,
            nonzero,
        );

        cumulative::<Widened, u8>(
            &logged(&recorded, 1, 2_000, &payload),
            &mut kept,
            &mut complaints,
            |_: &Widened, _: &mut Vec<u8>| unreachable!("nothing of another width decodes"),
        );

        assert_eq!(
            kept.iter().map(|row| row.message).collect::<Vec<_>>(),
            vec![1, 2],
            "the rows the last readable copy carried are still what the stream holds",
        );
        assert_eq!(complaints.len(), 1, "{complaints:?}");
        assert!(
            complaints[0].contains(SPOKEN) && complaints[0].contains("2000"),
            "the complaint names the channel and the instant, like every other one: {complaints:?}"
        );
    }

    #[test]
    fn a_folded_copy_that_does_not_decode_is_a_complaint_and_is_never_handed_on() {
        // The folding sibling's own refusal. A consumer of `each` keeps no
        // sample, so a message handed to its closure regardless of the decode
        // would fold arithmetic out of bytes read as the wrong message and
        // leave nothing behind saying it had happened.
        let recorded = ChannelMetadata::for_schema::<Sample>(SPOKEN);
        let payload = [1u8, 2, 0, 0, 0, 0, 0, 0];
        let mut complaints = Complaints::new();
        let mut taken = 0usize;

        each::<Widened>(
            &logged(&recorded, 1, 2_000, &payload),
            &mut complaints,
            |_| taken += 1,
        );

        assert_eq!(taken, 0, "nothing of another width decodes");
        assert_eq!(complaints.len(), 1, "{complaints:?}");
        assert!(
            complaints[0].contains(SPOKEN) && complaints[0].contains("2000"),
            "the complaint names the channel and the instant, like every other one: {complaints:?}"
        );
    }

    #[test]
    fn a_directory_with_no_log_in_it_is_refused_rather_than_read_as_empty() {
        let dir = scratch("empty");

        let refused = read_with::<Read>(&dir, &table());

        assert!(
            matches!(refused, Err(LogError::UnknownLogFormat { .. })),
            "a log whose format nothing here knows is refused: an empty channel set would skip \
             every binding check and decode payloads unexamined"
        );
    }
}
