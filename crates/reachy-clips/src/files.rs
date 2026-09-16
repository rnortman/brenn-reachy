//! What counts as an asset document on disk, and how a directory of them is
//! read.
//!
//! The one place the rule lives. A library directory is not only documents: the
//! importer copies each recording's sound sidecar in beside the clip it belongs
//! to, so a walk selects rather than takes everything. A clip library is also a
//! tree — an imported set gets its own subdirectory — so the walk descends where
//! the caller says to. And two documents claiming one name have to be decided by
//! something nobody has to guess at, so the order is the full path's and not the
//! filesystem's.
//!
//! **Both libraries walk through here.** The extension and the descent are the
//! caller's — a clip library is `json` and a tree, a pose library is
//! `textproto` and flat — because what changes between them is those two facts
//! and nothing about the rule: which entries count, and the sort that decides
//! an asset's id. Two walks would be two answers to that, and an emitter whose
//! two libraries were numbered by different rules renumbers one of them against
//! the sidecar that indexes both. This module sitting in the clip crate is the
//! same seam `TODO(clips-authoring-split)` names.
//!
//! The only host-side I/O in the crate besides the importer binary. It is here
//! rather than in each consumer because the daemon, the bench and the importer
//! all have to agree about which files are assets, and three copies of that
//! agreement diverge silently: a clip that plays on the bench and is missing
//! from the daemon's library is a bug nobody sees until a script names it.
//! Reporting stays with the caller — what to say about a file that will not
//! read is a daemon's, an operator's and a batch tool's own business.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// The extension a motion document carries.
pub const DOCUMENT_EXT: &str = "json";

/// Whether a walk goes into the subdirectories it finds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Descend {
    /// Into every subdirectory, at any depth: a library that grows a set at a
    /// time.
    Yes,
    /// The named directory only: a library whose documents are authored one at
    /// a time and all live together.
    No,
}

/// Every `ext` document under `dir`, by full path, ascending.
///
/// The order is the full path's, so where a document sits decides its id and
/// adding a subdirectory does not renumber what sorts before it. With
/// [`Descend::Yes`] an imported set can live in its own subdirectory so two sets
/// carry the same stem, and the hand-written documents stay at the top.
///
/// An unreadable directory is the error; an unreadable *file* is not this
/// function's business, since it has not read one. An entry whose name carries
/// `ext` is a document whatever it is on disk — a directory so named comes back
/// as a path that will not read, rather than being descended into.
pub fn document_paths(dir: &Path, ext: &str, descend: Descend) -> io::Result<Vec<PathBuf>> {
    let mut paths: Vec<PathBuf> = Vec::new();
    collect(dir, ext, descend, &mut paths)?;
    paths.sort();
    Ok(paths)
}

/// Append the documents under `dir` to `paths`.
fn collect(dir: &Path, ext: &str, descend: Descend, paths: &mut Vec<PathBuf>) -> io::Result<()> {
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.extension().is_some_and(|found| found == ext) {
            paths.push(path);
        } else if descend == Descend::Yes && entry.file_type()?.is_dir() {
            collect(&path, ext, descend, paths)?;
        }
    }
    Ok(())
}

/// Every `ext` document in `dir` as `(path, text-or-why-not)`, ascending.
///
/// One file that will not read is carried as its own error rather than failing
/// the walk: that is a skip like a document that will not validate, and a
/// library missing one motion is worth more than no library at all. A directory
/// that will not read is the error, because that is the caller's own
/// configuration being wrong.
pub fn documents(
    dir: &Path,
    ext: &str,
    descend: Descend,
) -> io::Result<Vec<(String, io::Result<String>)>> {
    Ok(document_paths(dir, ext, descend)?
        .into_iter()
        .map(|path| {
            let text = fs::read_to_string(&path);
            (path.display().to_string(), text)
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use reachy_scratch::scratch_dir;

    use super::*;

    /// The walk takes the JSON and leaves everything else, in path order.
    #[test]
    fn the_walk_selects_documents_and_sorts_them() {
        let dir = scratch_dir("reachy-clips-files-selects");
        for name in ["b.json", "a.json", "a.wav", "notes.txt"] {
            fs::write(dir.join(name), "{}").expect("written");
        }
        let paths =
            document_paths(dir.as_ref(), DOCUMENT_EXT, Descend::Yes).expect("the directory reads");
        let names: Vec<String> = paths
            .iter()
            .map(|path| path.file_name().expect("a name").to_string_lossy().into())
            .collect();
        assert_eq!(names, vec!["a.json".to_owned(), "b.json".to_owned()]);
    }

    /// A set in a subdirectory is part of the library, at any depth, and the
    /// order is the full path's rather than the name's — which is what makes an
    /// id stable when a set is added beside another.
    #[test]
    fn the_walk_descends_and_orders_by_full_path() {
        let dir = scratch_dir("reachy-clips-files-descends");
        let nested = dir.join("pollen").join("emotions");
        fs::create_dir_all(&nested).expect("a nested set");
        fs::create_dir_all(dir.join("sounds")).expect("a directory with no documents");
        for path in [
            dir.join("nod.json"),
            dir.join("pollen").join("index.json"),
            nested.join("bored1.json"),
            nested.join("bored1.ogg"),
            dir.join("sounds").join("hum.ogg"),
        ] {
            fs::write(path, "{}").expect("written");
        }
        let found =
            document_paths(dir.as_ref(), DOCUMENT_EXT, Descend::Yes).expect("the directory reads");
        let relative: Vec<String> = found
            .iter()
            .map(|path| {
                path.strip_prefix(&dir)
                    .expect("under the scratch root")
                    .to_string_lossy()
                    .into()
            })
            .collect();
        assert_eq!(
            relative,
            vec![
                "nod.json".to_owned(),
                "pollen/emotions/bored1.json".to_owned(),
                "pollen/index.json".to_owned(),
            ],
            "the .ogg files and the document-free directory are not documents"
        );
    }

    /// A flat walk takes the named directory and nothing under it, and the
    /// extension is the caller's: the pose library is authored that way.
    #[test]
    fn a_flat_walk_stays_in_the_directory_it_was_given() {
        let dir = scratch_dir("reachy-clips-files-flat");
        let nested = dir.join("drafts");
        fs::create_dir_all(&nested).expect("a subdirectory");
        for path in [
            dir.join("stow.textproto"),
            dir.join("neutral.textproto"),
            dir.join("notes.txt"),
            nested.join("peek.textproto"),
        ] {
            fs::write(path, "name: \"x\"").expect("written");
        }
        let found = document_paths(dir.as_ref(), "textproto", Descend::No).expect("it reads");
        let names: Vec<String> = found
            .iter()
            .map(|path| path.file_name().expect("a name").to_string_lossy().into())
            .collect();
        assert_eq!(
            names,
            vec!["neutral.textproto".to_owned(), "stow.textproto".to_owned()],
            "the subdirectory's document and the note are not this library's"
        );
    }

    /// A file that will not read is carried, not fatal; a directory that will
    /// not read is fatal.
    #[test]
    fn an_unreadable_file_is_carried_and_an_unreadable_directory_is_not() {
        let dir = scratch_dir("reachy-clips-files-unreadable");
        fs::write(dir.join("good.json"), "{\"kind\": \"clip\"}").expect("written");
        // A directory named like a document: opening it as a file fails, which
        // is the error shape a caller reports as a skip.
        fs::create_dir_all(dir.join("bad.json")).expect("created");

        let read =
            documents(dir.as_ref(), DOCUMENT_EXT, Descend::Yes).expect("the directory reads");
        assert_eq!(read.len(), 2);
        assert!(read[0].0.ends_with("bad.json"), "{:?}", read[0].0);
        assert!(read[0].1.is_err(), "a directory does not read as a file");
        assert_eq!(read[1].1.as_deref().expect("read"), "{\"kind\": \"clip\"}");

        assert!(documents(&dir.join("nowhere"), DOCUMENT_EXT, Descend::Yes).is_err());
    }
}
