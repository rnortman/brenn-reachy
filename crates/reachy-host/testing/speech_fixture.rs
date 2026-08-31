//! Speech configurations this host will actually run, written into a directory.
//!
//! The library's own cases and the binary's are separate crates, and both
//! compose the pod platform's server over a file on disk. One statement of what
//! such a file looks like, here, so the shape that makes
//! `Config::carries_alerts()` true is spelled once -- the tree asks that
//! question through `speech-surface` in one place, and a pin bump that narrows
//! or widens the answer must move one fixture, not two that can disagree.
//!
//! Test support only. Nothing a unit runs links this.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};

/// A key, as the pod platform's table spells one.
const KEY: &str = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";

/// Where a fixture's event stream goes.
///
/// The two callers differ here and nowhere else: one reads the file back to
/// prove the sink was opened and flushed, the other has no use for it.
#[derive(Debug, Clone, Copy)]
pub enum Events<'a> {
    /// No sink at all -- the platform's own word for it.
    Dropped,
    /// One JSONL file, at this path.
    File(&'a Path),
}

impl Events<'_> {
    /// The `sink` value, spelled as the configuration spells it.
    fn sink(self) -> String {
        match self {
            Events::Dropped => String::from("\"none\""),
            Events::File(path) => quoted(path),
        }
    }
}

/// `path` as a TOML basic string.
///
/// Every path this fixture writes into a configuration goes through here, so a
/// path the loader would read back as something else — or refuse — fails at the
/// fixture, naming the path, instead of surfacing as a parse error in whatever
/// case happened to compose it. Rust's `Debug` escaping and TOML's agree only
/// for text that needs no escape at all, so anything needing one is refused
/// rather than escaped: a temporary directory is the test machine's, and one
/// carrying a quote, a backslash, a control character or non-UTF-8 bytes is a
/// machine this fixture cannot write a configuration for.
///
/// # Panics
///
/// If the path is not UTF-8 or carries a character TOML would spell differently.
fn quoted(path: &Path) -> String {
    let text = path
        .to_str()
        .unwrap_or_else(|| panic!("a path this fixture can write into TOML: {path:?}"));
    assert!(
        !text.contains(['"', '\\']) && !text.contains(char::is_control),
        "a path this fixture can write into TOML unescaped: {text:?}"
    );
    format!("\"{text}\"")
}

/// A speech configuration the pod platform will run, written into `dir`.
///
/// Everything it names is inside that directory and nothing it names is on a
/// network: an ephemeral loopback listener, a key table of one pod, and no
/// recording. No `[wake]` and no `[endpointer]` table, so no model is loaded and
/// no inference runs -- what this composes is the server.
///
/// # Panics
///
/// If the directory cannot be written, which is a test that cannot use the
/// machine it is running on.
pub fn runnable(dir: &Path, events: Events<'_>) -> PathBuf {
    let keys = dir.join("psk.toml");
    speech_surface::psk::write_secret_file(&keys, &format!("fixture-pod = \"{KEY}\"\n"))
        .expect("a key table");
    let path = dir.join("speech.toml");
    std::fs::write(
        &path,
        format!(
            "listen_addr = \"127.0.0.1:0\"\n\
             pod_psk_file = {}\n\
             [record]\nenabled = false\n\
             [jsonl]\nsink = {}\n",
            quoted(&keys),
            events.sink(),
        ),
    )
    .expect("a file");
    path
}

/// The same fixture with a bus brain: the deployment whose alerts travel.
///
/// Everything `mode = "brenn"` requires and nothing that dials anybody: the
/// speech services are URLs a composition that runs no turn never calls, and
/// the bridge is pointed at a closed loopback port with a token file written
/// here. What it buys is the run that drains the alert seam.
///
/// # Panics
///
/// If the directory cannot be written.
pub fn carrying(dir: &Path, events: Events<'_>) -> PathBuf {
    let token = dir.join("bus.token");
    speech_surface::psk::write_secret_file(&token, "a-bearer-token\n").expect("a token file");
    let path = runnable(dir, events);
    let text = std::fs::read_to_string(&path).expect("the fixture");
    std::fs::write(
        &path,
        format!(
            "{text}\
             [brain]\nmode = \"brenn\"\n\
             [stt]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\nmodel = \"m\"\n\
             [tts]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\n\
             model = \"m\"\nvoice = \"v\"\n\
             [brenn]\n\
             publish_channel = \"brenn:pod.utterance\"\n\
             response_channel = \"brenn:pod.speak\"\n\
             [brenn.bridge]\n\
             server_url = \"wss://127.0.0.1:1/ws\"\n\
             token_file = {}\n",
            quoted(&token),
        ),
    )
    .expect("a file");
    path
}
