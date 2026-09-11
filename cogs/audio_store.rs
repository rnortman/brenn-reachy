//! The fetched audio store, read the one way both analyzers of a fetch read it.
//!
//! A run brings home a `.audio` directory of the pod's frame logs, and both
//! offline analyzers cut `.wav` files out of it: the speech run report one per
//! turn, the pose session report one per utterance. Same store, same format,
//! same resolver, same three ways it can fail — so it is one implementation
//! rather than one per tool, for the reason `run_report::recover` is: a store
//! outage one of them calls a fault and the other calls a session where nobody
//! spoke is two tools disagreeing about what a run did.
//!
//! What a tool keeps for itself is what it does with the audio. What it gets
//! from here is whether the store holds anything, the resolve of one span with
//! its faults said in one set of words, and the write of the decoded samples.
//!
//! The resolver and the wav writer are brenn-pod's own: the store's format has
//! one reader in the world and a second one here would be a copy to keep in
//! step with a format this tree does not own.

#![forbid(unsafe_code)]

use std::path::Path;

use run_report::quote;

/// Whether the fetched store holds anything at all.
///
/// Three states, not two, and the third is why this is not an `is_dir` at the
/// call site: a site whose configuration records nothing leaves an empty
/// directory behind (the fetch's rsync of an empty store succeeds), a fetch
/// that brought no audio home leaves none, and a store that will not open is
/// neither. `Ok(false)` is "nothing was recorded"; `Err` is the fault, in its
/// own words. An outage must never read as a session where nobody spoke.
///
/// # Errors
///
/// The store exists and could not be read.
pub fn recorded(store: &Path) -> Result<bool, String> {
    match std::fs::read_dir(store) {
        Ok(mut entries) => Ok(entries.any(|entry| entry.is_ok())),
        Err(why) if why.kind() == std::io::ErrorKind::NotFound => Ok(false),
        Err(why) => Err(format!("{}: {why}", store.display())),
    }
}

/// Decode one span of one frame log out of the store.
///
/// The span names no covering segment: this code assumes the resolver treats
/// segment refs as provenance rather than a filter, and a console leaves them
/// empty on exactly the carves whose recogniser ran long. Uncovered stretches
/// inside a span resolve
/// as silence, which is the pruner's business and not a failure; a log the
/// pruner took entirely is not in the store and says so.
///
/// The log name is console-authored text and is quoted wherever it is said: a
/// name carrying a newline would otherwise fabricate a line of a report.
///
/// TODO(clip-one-pass-per-log): every carve of a run comes out of the same log
/// and each call here decodes it from the head, so a session's cuts cost work
/// quadratic in the number of things said in it.
///
/// # Errors
///
/// The sample bounds are not a span, the resolver would not read the log, or
/// the log is not in the store.
pub fn resolve(
    store: &Path,
    log: &str,
    start: i64,
    end: i64,
) -> Result<speech_pipeline::ResolvedSpanAudio, String> {
    // A sample index off a line that tore is a number this does not carve
    // with: the same fact the resolver names `InvalidSpan`, said in the same
    // words, rather than a panic in the report a run is read by.
    let (Ok(start_sample), Ok(end_sample)) = (u64::try_from(start), u64::try_from(end)) else {
        return Err("invalid span".to_owned());
    };
    let span = speech_pipeline::AudioSpan {
        log: log.to_owned(),
        start_sample,
        end_sample,
        segments: Vec::new(),
    };
    let audio = match span.resolve(store) {
        Ok(audio) => audio,
        Err(speech_pipeline::SpanResolveError::InvalidSpan { .. }) => {
            return Err("invalid span".to_owned());
        }
        Err(speech_pipeline::SpanResolveError::Resolve { log, source }) => {
            return Err(format!("{}: {source}", quote(&log)));
        }
    };
    if audio.pruned.iter().any(|part| part.log == log) {
        return Err(format!("{} is not in the store", quote(log)));
    }
    Ok(audio)
}

/// Write decoded samples into `into` as `name`.
///
/// `made` carries the one attempt at creating the output directory, made by the
/// first caller that gets as far as audio to write: a directory that cannot be
/// made is one fact about the filesystem rather than one refusal per cut.
///
/// # Errors
///
/// The directory could not be made, or the file could not be written.
pub fn write_cut(
    into: &Path,
    name: &str,
    pcm: &[i16],
    made: &mut Option<Result<(), String>>,
) -> Result<(), String> {
    if let Err(why) = made.get_or_insert_with(|| {
        std::fs::create_dir_all(into).map_err(|why| format!("{}: {why}", into.display()))
    }) {
        return Err(why.clone());
    }
    speech_pipeline::write_spine_wav(&into.join(name), pcm).map_err(|why| format!("{name}: {why}"))
}

#[cfg(test)]
mod tests {
    use reachy_scratch::scratch_dir;

    use super::{recorded, resolve, write_cut};

    #[test]
    fn a_store_that_is_not_there_recorded_nothing_and_one_that_is_empty_did_too() {
        let at = scratch_dir("run-audio-empty");
        assert_eq!(recorded(&at.join("absent")), Ok(false));
        std::fs::create_dir_all(at.join("present")).expect("a store");
        assert_eq!(recorded(&at.join("present")), Ok(false));
        std::fs::write(at.join("present").join("a.framelog"), b"x").expect("a log");
        assert_eq!(recorded(&at.join("present")), Ok(true));
    }

    #[test]
    fn a_store_that_will_not_open_is_a_fault_and_not_an_empty_one() {
        let at = scratch_dir("run-audio-file-as-store");
        // A file where the store should be is the shape of every store that
        // will not open, and the one a test can make without privileges.
        std::fs::write(at.join("store"), b"not a directory").expect("a file");
        let why = recorded(&at.join("store")).expect_err("a fault");
        assert!(why.contains("store"), "{why}");
    }

    #[test]
    fn sample_bounds_that_are_not_a_span_are_refused_before_the_store_is_touched() {
        let at = scratch_dir("run-audio-bounds");
        assert_eq!(
            resolve(&at.join("nowhere"), "a.framelog", -1, 10),
            Err("invalid span".to_owned())
        );
        assert_eq!(
            resolve(&at.join("nowhere"), "a.framelog", 10, -1),
            Err("invalid span".to_owned())
        );
    }

    #[test]
    fn a_log_the_store_does_not_hold_is_said_with_its_name_quoted() {
        let at = scratch_dir("run-audio-missing-log");
        std::fs::create_dir_all(at.join("store")).expect("a store");
        let why = resolve(&at.join("store"), "a.framelog\nnot a line", 0, 16_000)
            .expect_err("no such log");
        assert!(!why.contains('\n'), "{why}");
    }

    #[test]
    fn the_output_directory_is_made_once_and_its_failure_is_remembered() {
        let at = scratch_dir("run-audio-write");
        let into = at.join("cuts");
        let mut made = None;
        write_cut(&into, "one.wav", &[0_i16; 16], &mut made).expect("a wav");
        assert!(into.join("one.wav").is_file());
        write_cut(&into, "two.wav", &[0_i16; 16], &mut made).expect("a second wav");
        assert!(into.join("two.wav").is_file());

        // A file where the directory should be: the first attempt fails, and
        // the second says the same thing without trying again.
        std::fs::write(at.join("blocked"), b"x").expect("a file");
        let mut blocked = None;
        let first = write_cut(&at.join("blocked"), "one.wav", &[0_i16; 16], &mut blocked)
            .expect_err("no directory");
        let again = write_cut(&at.join("blocked"), "two.wav", &[0_i16; 16], &mut blocked)
            .expect_err("still no directory");
        assert_eq!(first, again);
    }
}
