//! `--check`: what this host would load, decided without a robot.
//!
//! A speech run's inputs are two configurations and a handful of files they
//! name by path, and every one of those paths is resolved against the process's
//! working directory — which on a unit is the payload root. Nothing about that
//! resolution needs the unit: the same binary, run over the staged payload with
//! that directory as its own, answers the same question the launcher would, and
//! answers it before anything is pushed or torqued.
//!
//! What is decided here is loading and presence, and nothing else. The
//! configurations go through their real readers, so a typo is the loader's own
//! refusal rather than a second opinion this module maintains; every path field
//! the loaded configurations name is then looked for where the run would look.
//! No socket is opened, no model is read, and no file's contents are printed —
//! two of the paths below name secrets.
//!
//! One rule here is stricter than the loader's: a path in the speech
//! configuration must be relative. Every file that configuration names travels
//! inside the payload and is named payload-relative, so an absolute path is a
//! workstation-era leftover — it would be found here and be absent on the unit,
//! which is exactly the migration failure this preflight exists to catch. The
//! host's own configuration is not held to that: the launcher and an operator
//! both name it wherever it is.

use std::path::{Path, PathBuf};

use clockwork_rs::SyncTime;
use serde_json::json;

use crate::params;

/// One thing the check looked at, and what it found.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Conclusion {
    /// What kind of conclusion this is, as the line's `kind`.
    pub kind: &'static str,
    /// What it is about: a configuration's path, or the field naming a file.
    pub subject: String,
    /// Whether this conclusion is one a run can proceed on.
    pub held: bool,
    /// The sentence a person reads.
    pub says: String,
}

/// Look at both configurations and everything they name, relative to `base`.
///
/// `base` is the directory a relative path resolves against — the process's own
/// working directory on a real run, which is [`Path::new("")`], and a scratch
/// directory in a test. The check is over the configurations as written: a
/// speech configuration that was not named is a legal host and not a finding,
/// because the edge half runs alone on a unit whose payload carries no
/// pipeline inputs.
///
/// The last conclusion is the verdict, so a reader of the stream has the answer
/// without accumulating the lines before it.
#[must_use]
pub fn inspect(config: &Path, speech_config: Option<&Path>, base: &Path) -> Vec<Conclusion> {
    let mut found = Vec::new();
    match params::load(config) {
        Ok(settings) => {
            found.push(Conclusion {
                kind: "params",
                subject: config.display().to_string(),
                held: true,
                says: format!(
                    "{} is a host configuration this build reads, answering for `{}`",
                    config.display(),
                    settings.edge.pod(),
                ),
            });
            // Named by the host's own configuration rather than the speech
            // one, so the relative-path rule above does not apply to it: what
            // it must be is there.
            found.push(present("clip_names_path", &settings.clip_names, base));
        }
        Err(error) => found.push(Conclusion {
            kind: "params",
            subject: config.display().to_string(),
            held: false,
            says: error.to_string(),
        }),
    }

    match speech_config {
        None => found.push(Conclusion {
            kind: crate::words::VOICELESS,
            subject: String::new(),
            held: true,
            says: "no speech configuration was named, so this host is its edge half alone"
                .to_owned(),
        }),
        Some(path) => found.extend(speech(path, base)),
    }

    let verdict = verdict(&found);
    found.push(verdict);
    found
}

/// Whether every conclusion held.
#[must_use]
pub fn settled(found: &[Conclusion]) -> bool {
    found.iter().all(|conclusion| conclusion.held)
}

/// One conclusion, as a line of JSON.
///
/// Its own stream rather than the edge's: these lines are a preflight's answer
/// on a workstation, and nothing that reads a run's log should have to tell
/// them from the narration of a session that happened.
#[must_use]
pub fn conclusion_line(conclusion: &Conclusion, at: SyncTime) -> String {
    json!({
        "stream": "check",
        "at_ns": at.as_nanos(),
        "kind": conclusion.kind,
        "subject": conclusion.subject,
        "held": conclusion.held,
        "says": one_line(&conclusion.says),
    })
    .to_string()
}

/// The speech configuration, and every file it names.
///
/// A configuration that will not load ends the walk: the path fields below are
/// read off the loaded value, and there is no partial value to read them from.
fn speech(path: &Path, base: &Path) -> Vec<Conclusion> {
    let config = match speech_surface::Config::load(path) {
        Ok(config) => config,
        Err(error) => {
            return vec![Conclusion {
                kind: "speech_config",
                subject: path.display().to_string(),
                held: false,
                says: error.to_string(),
            }];
        }
    };
    let mut found = vec![Conclusion {
        kind: "speech_config",
        subject: path.display().to_string(),
        held: true,
        says: format!(
            "{} is a speech configuration the pipeline will run on; it listens on {}",
            path.display(),
            config.listen_addr,
        ),
    }];
    for (field, named) in named_paths(&config) {
        found.push(if named.is_absolute() {
            Conclusion {
                kind: "absolute",
                subject: field.to_owned(),
                held: false,
                says: format!(
                    "`{field}` is the absolute path {} — every file a speech configuration names \
                     travels inside the payload and is named relative to it, so an absolute path \
                     is one the unit will not find",
                    named.display(),
                ),
            }
        } else {
            present(field, &named, base)
        });
    }
    found
}

/// Every path field a loaded speech configuration states, with its field name.
///
/// An absent table names nothing, which is how a voiced-but-bus-less
/// deployment, or one with no wake gate, reads here.
fn named_paths(config: &speech_surface::Config) -> Vec<(&'static str, PathBuf)> {
    let mut named = vec![("pod_psk_file", config.pod_psk_file.clone())];
    if let Some(brenn) = &config.brenn {
        named.push(("brenn.bridge.token_file", brenn.bridge.token_file.clone()));
    }
    if let Some(wake) = &config.wake {
        for (field, path) in [
            ("wake.melspectrogram", &wake.melspectrogram),
            ("wake.embedding", &wake.embedding),
            ("wake.model", &wake.model),
        ] {
            if let Some(path) = path {
                named.push((field, path.clone()));
            }
        }
    }
    if let Some(endpointer) = &config.endpointer {
        named.push(("endpointer.model", endpointer.model.clone()));
    }
    if let Some(brain) = &config.brain
        && let Some(clip) = &brain.clip
    {
        named.push(("brain.clip", clip.clone()));
    }
    named
}

/// Whether the file a field names is where the run will look for it.
///
/// A directory at that name is a finding of its own: it reads as present to
/// anything that only asks whether the path exists, and every one of these is
/// opened as a file.
fn present(field: &str, named: &Path, base: &Path) -> Conclusion {
    let at = base.join(named);
    let says = match std::fs::metadata(&at) {
        Ok(found) if found.is_file() => {
            return Conclusion {
                kind: "names",
                subject: field.to_owned(),
                held: true,
                says: format!("`{field}` names {}, which is there", at.display()),
            };
        }
        Ok(_) => format!("`{field}` names {}, which is not a file", at.display()),
        Err(error) => format!("`{field}` names {}, which {error}", at.display()),
    };
    Conclusion {
        kind: "names",
        subject: field.to_owned(),
        held: false,
        says,
    }
}

/// The answer, from everything looked at so far.
fn verdict(found: &[Conclusion]) -> Conclusion {
    let unheld: Vec<&str> = found
        .iter()
        .filter(|conclusion| !conclusion.held)
        .map(|conclusion| conclusion.subject.as_str())
        .collect();
    if unheld.is_empty() {
        Conclusion {
            kind: "checked",
            subject: String::new(),
            held: true,
            says: "both configurations load and every file they name is where the run will look \
                   for it"
                .to_owned(),
        }
    } else {
        Conclusion {
            kind: "checked",
            subject: String::new(),
            held: false,
            says: format!(
                "this configuration is not one a run can start on: {}",
                unheld.join(", ")
            ),
        }
    }
}

/// `text` with its line breaks flattened, so one conclusion is one line.
///
/// A loader's refusal is often several lines, and a JSON string carries them as
/// escapes — legal, and unreadable in a terminal tailing the check.
fn one_line(text: &str) -> String {
    text.split_whitespace().collect::<Vec<&str>>().join(" ")
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use clockwork_rs::SyncTime;
    use reachy_scratch::scratch_dir;

    use super::{Conclusion, conclusion_line, inspect, settled};

    /// A host configuration this build reads, naming `clip_names_path`.
    fn params(dir: &Path, clip_names: &str) -> PathBuf {
        let path = dir.join("host_params.textproto");
        std::fs::write(
            &path,
            format!(
                "pod: \"fixture-reachy\"\n\
                 stow_duration_ms: 3000\n\
                 body_cap_bytes: 8192\n\
                 clip_names_path: \"{clip_names}\"\n",
            ),
        )
        .expect("a file");
        path
    }

    /// A clip name table, at `name` inside `dir`.
    fn clip_names(dir: &Path, name: &str) -> PathBuf {
        let path = dir.join(name);
        std::fs::write(&path, "{\"names\": []}\n").expect("a file");
        path
    }

    /// The conclusion about `subject`, which the case expects to be there.
    fn about<'a>(found: &'a [Conclusion], subject: &str) -> &'a Conclusion {
        found
            .iter()
            .find(|conclusion| conclusion.subject == subject)
            .unwrap_or_else(|| panic!("a conclusion about `{subject}`: {found:?}"))
    }

    /// The verdict, which is always the last conclusion.
    fn verdict(found: &[Conclusion]) -> &Conclusion {
        found.last().expect("a verdict")
    }

    #[test]
    fn a_configuration_whose_files_are_all_there_is_one_a_run_can_start_on() {
        let dir = scratch_dir("reachy-host-check-clean");
        let config = params(dir.as_ref(), "clip_library.names.json");
        clip_names(dir.as_ref(), "clip_library.names.json");
        let speech = speech_fixture::carrying_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(settled(&found), "{found:?}");
        assert!(about(&found, "pod_psk_file").held, "{found:?}");
        assert!(about(&found, "brenn.bridge.token_file").held, "{found:?}");
        assert!(about(&found, "clip_names_path").held, "{found:?}");
        assert_eq!(verdict(&found).kind, "checked");
    }

    #[test]
    fn a_configuration_with_no_bus_names_no_token() {
        // The voiced, bus-less deployment: legal, and the walk simply has one
        // fewer file to find.
        let dir = scratch_dir("reachy-host-check-busless");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::runnable_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(settled(&found), "{found:?}");
        assert!(
            !found
                .iter()
                .any(|conclusion| conclusion.subject == "brenn.bridge.token_file"),
            "{found:?}",
        );
    }

    /// Each model-and-clip field of `modelled_named`, with the file it names.
    const MODELLED: [(&str, &str); 5] = [
        ("wake.melspectrogram", speech_fixture::WAKE_MELSPECTROGRAM),
        ("wake.embedding", speech_fixture::WAKE_EMBEDDING),
        ("wake.model", speech_fixture::WAKE_MODEL),
        ("endpointer.model", speech_fixture::ENDPOINTER_MODEL),
        ("brain.clip", speech_fixture::BRAIN_CLIP),
    ];

    #[test]
    fn every_model_a_configuration_names_is_looked_for_by_its_own_name() {
        // The largest files a payload carries and the ones most easily left out
        // of it. Each field names a different file here, so a walk that read
        // two of them off one struct member would say a name this does not
        // expect.
        let dir = scratch_dir("reachy-host-check-models");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::modelled_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(settled(&found), "{found:?}");
        for (field, file) in MODELLED {
            let names = about(&found, field);
            assert_eq!(names.kind, "names", "{names:?}");
            assert!(names.held, "{names:?}");
            assert!(
                names
                    .says
                    .contains(dir.join(file).to_str().expect("a path this fixture wrote")),
                "{names:?}",
            );
        }
    }

    #[test]
    fn a_model_the_payload_does_not_carry_is_the_only_thing_that_fails() {
        let dir = scratch_dir("reachy-host-check-model-missing");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::modelled_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        std::fs::remove_file(dir.join(speech_fixture::WAKE_EMBEDDING))
            .expect("the fixture's embedding model");

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        let missing = about(&found, "wake.embedding");
        assert!(!missing.held, "{missing:?}");
        assert!(
            missing.says.contains(
                dir.join(speech_fixture::WAKE_EMBEDDING)
                    .to_str()
                    .expect("a path this fixture wrote")
            ),
            "{missing:?}",
        );
        for (field, _) in MODELLED {
            if field != "wake.embedding" {
                assert!(about(&found, field).held, "{found:?}");
            }
        }
        assert!(verdict(&found).says.contains("wake.embedding"), "{found:?}");
    }

    #[test]
    fn a_key_table_that_is_not_there_is_named_and_refused() {
        let dir = scratch_dir("reachy-host-check-missing-psk");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::runnable_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        std::fs::remove_file(dir.join("psk.toml")).expect("the fixture's key table");

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        let names = about(&found, "pod_psk_file");
        assert!(!names.held, "{names:?}");
        assert!(
            names.says.contains(
                dir.join("psk.toml")
                    .to_str()
                    .expect("a path this case wrote")
            ),
            "{names:?}",
        );
        assert!(verdict(&found).says.contains("pod_psk_file"), "{found:?}");
    }

    #[test]
    fn an_absolute_path_is_refused_even_where_the_file_is_there() {
        // The workstation-era spelling: found here, absent on the unit. The
        // fixture's own naming is absolute and every file it names exists, so
        // this case is the migration error exactly.
        let dir = scratch_dir("reachy-host-check-absolute");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::carrying(dir.as_ref(), speech_fixture::Events::Dropped);

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        for field in ["pod_psk_file", "brenn.bridge.token_file"] {
            let refused = about(&found, field);
            assert_eq!(refused.kind, "absolute", "{refused:?}");
            assert!(refused.says.contains("inside the payload"), "{refused:?}");
        }
    }

    #[test]
    fn a_directory_where_a_file_is_named_is_not_a_file() {
        let dir = scratch_dir("reachy-host-check-directory");
        let config = params(dir.as_ref(), "names.json");
        std::fs::create_dir(dir.join("names.json")).expect("a directory in the way");

        let found = inspect(&config, None, dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        assert!(
            about(&found, "clip_names_path").says.contains("not a file"),
            "{found:?}",
        );
    }

    #[test]
    fn an_unknown_key_is_the_loader_s_own_refusal() {
        let dir = scratch_dir("reachy-host-check-unknown-key");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::runnable_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        let text = std::fs::read_to_string(&speech).expect("the fixture");
        std::fs::write(&speech, format!("listen_adr = \"127.0.0.1:0\"\n{text}")).expect("a file");

        let found = inspect(&config, Some(&speech), dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        let refused = about(&found, &speech.display().to_string());
        assert_eq!(refused.kind, "speech_config");
        assert!(refused.says.contains("listen_adr"), "{refused:?}");
        assert!(
            !found
                .iter()
                .any(|conclusion| conclusion.subject == "pod_psk_file"),
            "a configuration that will not load names nothing to look for: {found:?}",
        );
    }

    #[test]
    fn a_host_configuration_that_will_not_read_is_the_reader_s_own_refusal() {
        let dir = scratch_dir("reachy-host-check-bad-params");
        let config = dir.join("host_params.textproto");
        std::fs::write(&config, "pod: \"fixture-reachy\"\nnot_a_field: 1\n").expect("a file");

        let found = inspect(&config, None, dir.as_ref());
        assert!(!settled(&found), "{found:?}");
        let refused = about(&found, &config.display().to_string());
        assert_eq!(refused.kind, "params");
        assert!(refused.says.contains("not_a_field"), "{refused:?}");
    }

    #[test]
    fn a_host_asked_for_no_speech_configuration_is_still_a_host() {
        let dir = scratch_dir("reachy-host-check-voiceless");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");

        let found = inspect(&config, None, dir.as_ref());
        assert!(settled(&found), "{found:?}");
        assert!(
            found
                .iter()
                .any(|conclusion| conclusion.kind == "voiceless"),
            "{found:?}",
        );
    }

    #[test]
    fn a_conclusion_is_one_line_of_json_on_its_own_stream() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = conclusion_line(
            &Conclusion {
                kind: "names",
                subject: "pod_psk_file".to_owned(),
                held: false,
                says: "a refusal\nwritten over\ntwo lines".to_owned(),
            },
            at,
        );
        assert!(!line.contains('\n'), "{line}");
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["stream"], "check");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["kind"], "names");
        assert_eq!(parsed["subject"], "pod_psk_file");
        assert_eq!(parsed["held"], false);
        assert_eq!(parsed["says"], "a refusal written over two lines");
    }

    #[test]
    fn nothing_the_check_prints_is_a_file_s_contents() {
        // Two of the paths it stats name secrets. What a line carries about one
        // is its field name and where it is, and this case is what says so.
        let dir = scratch_dir("reachy-host-check-secrets");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");
        let speech = speech_fixture::carrying_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        let key = std::fs::read_to_string(dir.join("psk.toml")).expect("the key table");
        let token = std::fs::read_to_string(dir.join("bus.token")).expect("the token");

        let at = SyncTime::from_nanos(1);
        let printed: String = inspect(&config, Some(&speech), dir.as_ref())
            .iter()
            .map(|conclusion| conclusion_line(conclusion, at))
            .collect();
        for secret in [key.trim(), token.trim()] {
            assert!(!printed.contains(secret), "a secret reached the stream");
        }
    }

    #[test]
    fn a_relative_base_is_the_working_directory_the_run_resolves_against() {
        // What the binary passes: the empty base, which joins to the path as
        // written and so resolves against this process's own directory.
        let dir = scratch_dir("reachy-host-check-cwd-base");
        let config = params(dir.as_ref(), "names.json");
        clip_names(dir.as_ref(), "names.json");

        let found = inspect(&config, None, Path::new(""));
        assert!(
            !settled(&found),
            "the table is in the scratch directory, not this process's: {found:?}",
        );
        assert!(
            about(&found, "clip_names_path")
                .says
                .contains("names names.json"),
            "{found:?}",
        );
    }
}
