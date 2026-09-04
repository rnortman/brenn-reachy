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

/// How a fixture spells the paths of the files it names.
///
/// A configuration written for a workstation names its files absolutely; one
/// written for the payload names them relative to the directory the host runs
/// in. Both are configurations the loader accepts, and the difference is what
/// the deployment preflight is about, so the fixture states it rather than
/// leaving each case to rewrite the file it was handed.
#[derive(Debug, Clone, Copy)]
pub enum Naming {
    /// Absolute paths, as a workstation's own configuration carries.
    Absolute,
    /// Paths relative to the directory the fixture is written into, which is
    /// what a payload member's configuration carries.
    PayloadRelative,
}

impl Naming {
    /// `path`, spelled the way this naming spells it. `name` is the file's name
    /// inside the fixture's directory.
    fn spell(self, path: &Path, name: &str) -> String {
        match self {
            Naming::Absolute => quoted(path),
            Naming::PayloadRelative => format!("\"{name}\""),
        }
    }
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
    runnable_named(dir, events, Naming::Absolute)
}

/// The same fixture, spelling the files it names the way `naming` says.
///
/// # Panics
///
/// If the directory cannot be written.
pub fn runnable_named(dir: &Path, events: Events<'_>, naming: Naming) -> PathBuf {
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
            naming.spell(&keys, "psk.toml"),
            events.sink(),
        ),
    )
    .expect("a file");
    path
}

/// The same fixture, naming `pods` in its `[pods]` table.
///
/// The table is the only set of pod ids a loaded configuration exposes, and it
/// is what the deployment preflight compares the host's own `pod` against, so
/// the fixture states it rather than leaving each case to append TOML to the
/// file it was handed. Every entry gets the same room: what the comparison
/// reads is the keys.
///
/// # Panics
///
/// If the directory cannot be written.
pub fn runnable_with_pods(
    dir: &Path,
    events: Events<'_>,
    naming: Naming,
    pods: &[&str],
) -> PathBuf {
    let path = runnable_named(dir, events, naming);
    let text = std::fs::read_to_string(&path).expect("the fixture");
    let mut table = String::new();
    for pod in pods {
        assert!(
            !pod.contains(['"', '\\', '[', ']', '.']) && !pod.contains(char::is_control),
            "a pod id this fixture can write as a TOML table key unquoted: {pod:?}"
        );
        table.push_str(&format!("[pods.{pod}]\nroom = \"fixture\"\n"));
    }
    std::fs::write(&path, format!("{text}{table}")).expect("a file");
    path
}

/// The same fixture with every model path a configuration can name.
///
/// A `[wake]` table with its three openWakeWord models, an `[endpointer]` with
/// the VAD model, and a `[brain]` answering every utterance with a clip: the
/// five path fields beside the two credential ones, each naming a file of its
/// own so a check that looks at the wrong struct member names the wrong file.
/// Every one of them is written, so a case that wants one missing removes it.
///
/// Nothing here is a real model — they are bytes at a path, which is all a
/// presence check reads. No `[brenn]`, so this composes without a bus.
///
/// # Panics
///
/// If the directory cannot be written.
pub fn modelled_named(dir: &Path, events: Events<'_>, naming: Naming) -> PathBuf {
    let path = runnable_named(dir, events, naming);
    let named = |name: &str| -> String {
        let at = dir.join(name);
        std::fs::write(&at, b"not a model").expect("a file");
        naming.spell(&at, name)
    };
    let melspectrogram = named(WAKE_MELSPECTROGRAM);
    let embedding = named(WAKE_EMBEDDING);
    let wake_model = named(WAKE_MODEL);
    let endpointer = named(ENDPOINTER_MODEL);
    let clip = named(BRAIN_CLIP);
    let text = std::fs::read_to_string(&path).expect("the fixture");
    std::fs::write(
        &path,
        format!(
            "{text}\
             [wake]\nmode = \"oww\"\n\
             melspectrogram = {melspectrogram}\n\
             embedding = {embedding}\n\
             model = {wake_model}\n\
             [endpointer]\nmodel = {endpointer}\n\
             [brain]\nmode = \"wav\"\nclip = {clip}\n",
        ),
    )
    .expect("a file");
    path
}

/// The file `wake.melspectrogram` names in [`modelled_named`].
pub const WAKE_MELSPECTROGRAM: &str = "wake-melspectrogram.onnx";

/// The file `wake.embedding` names in [`modelled_named`].
pub const WAKE_EMBEDDING: &str = "wake-embedding.onnx";

/// The file `wake.model` names in [`modelled_named`].
pub const WAKE_MODEL: &str = "wake-phrase.onnx";

/// The file `endpointer.model` names in [`modelled_named`].
pub const ENDPOINTER_MODEL: &str = "endpointer-vad.onnx";

/// The file `brain.clip` names in [`modelled_named`].
pub const BRAIN_CLIP: &str = "brain-answer.wav";

/// The recording block every written fixture carries, as `records` finds it.
const RECORDING_OFF: &str = "[record]\nenabled = false\n";

/// Turn a written fixture's recording on, into `dir` under `cap_bytes`.
///
/// The fixtures record nothing, because a composition that runs no turn has
/// nothing to record and a test that wrote a store would write it into the
/// machine it is running on. What a preflight says about recording is decided
/// by the switch and the cap together, so a case that is about that sentence
/// states both here rather than editing TOML of its own.
///
/// # Panics
///
/// If the file cannot be read back and rewritten, if it carries no recording
/// block for the switch to go into, or if `dir` is a path this fixture cannot
/// write into TOML.
pub fn records(config: &Path, dir: &Path, cap_bytes: u64) {
    let text = std::fs::read_to_string(config).expect("the fixture");
    let recording = format!(
        "[record]\nenabled = true\ndir = {}\ncap_bytes = {cap_bytes}\n",
        quoted(dir),
    );
    // The anchor is asserted rather than assumed: a case that turns recording
    // on and silently gets a fixture recording nothing still passes wherever it
    // asserts the *absence* of a conclusion, which is what the two cases
    // guarding the flash-write refusal do.
    assert!(
        text.contains(RECORDING_OFF),
        "a fixture with a recording block to turn on: {config:?}"
    );
    let text = text.replace(RECORDING_OFF, &recording);
    std::fs::write(config, text).expect("a file");
}

/// Give a written fixture's `[stt]` table the optional secondary gate.
///
/// The floor is off by default, so the sentence a preflight prints about the
/// gate has two shapes and this is what a case asks for the second one. Written
/// into the table rather than appended to the file: a key after the last table
/// belongs to that table, which here would be the bridge's.
///
/// # Panics
///
/// If the file cannot be read back and rewritten, or if it carries no `[stt]`
/// table for the key to go into.
pub fn declining_below(config: &Path, avg_logprob_min: f32) {
    let text = std::fs::read_to_string(config).expect("the fixture");
    assert!(
        text.contains("[stt]\n"),
        "a fixture with an [stt] table to put a gate floor in: {config:?}"
    );
    let text = text.replace(
        "[stt]\n",
        &format!("[stt]\navg_logprob_min = {avg_logprob_min:?}\n"),
    );
    std::fs::write(config, text).expect("a file");
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
    carrying_named(dir, events, Naming::Absolute)
}

/// The same fixture, spelling the files it names the way `naming` says.
///
/// # Panics
///
/// If the directory cannot be written.
pub fn carrying_named(dir: &Path, events: Events<'_>, naming: Naming) -> PathBuf {
    let token = dir.join("bus.token");
    speech_surface::psk::write_secret_file(&token, "a-bearer-token\n").expect("a token file");
    let path = runnable_named(dir, events, naming);
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
            naming.spell(&token, "bus.token"),
        ),
    )
    .expect("a file");
    path
}
