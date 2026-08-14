//! What counts as a motion document on disk, and how a directory of them is
//! read.
//!
//! The one place the rule lives. A library directory is not only motions: the
//! importer copies each recording's sound sidecar in beside the clip it belongs
//! to, so a walk selects rather than takes everything. And two documents
//! claiming one name have to be decided by something nobody has to guess at, so
//! the order is the path's and not the filesystem's.
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

/// Every motion document in `dir`, by path, ascending.
///
/// An unreadable directory is the error; an unreadable *file* is not this
/// function's business, since it has not read one.
pub fn document_paths(dir: &Path) -> io::Result<Vec<PathBuf>> {
    let mut paths: Vec<PathBuf> = Vec::new();
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if path.extension().is_some_and(|ext| ext == DOCUMENT_EXT) {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

/// Every motion document in `dir` as `(path, text-or-why-not)`, ascending.
///
/// One file that will not read is carried as its own error rather than failing
/// the walk: that is a skip like a document that will not validate, and a
/// library missing one motion is worth more than no library at all. A directory
/// that will not read is the error, because that is the caller's own
/// configuration being wrong.
pub fn documents(dir: &Path) -> io::Result<Vec<(String, io::Result<String>)>> {
    Ok(document_paths(dir)?
        .into_iter()
        .map(|path| {
            let text = fs::read_to_string(&path);
            (path.display().to_string(), text)
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicU32, Ordering};

    use super::*;

    /// A scratch directory of this test's own, named so one left behind by a
    /// panic says which test left it.
    fn scratch(name: &str) -> PathBuf {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "reachy-clips-files-{}-{}-{name}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&path).expect("a scratch directory");
        path
    }

    /// Remove a scratch directory a test is done with.
    ///
    /// Called on the passing path only: a failing test keeps its directory, so
    /// what it was looking at is still there to look at.
    fn swept(dir: &Path) {
        fs::remove_dir_all(dir).expect("the scratch directory goes away");
    }

    /// The walk takes the JSON and leaves everything else, in path order.
    #[test]
    fn the_walk_selects_documents_and_sorts_them() {
        let dir = scratch("selects");
        for name in ["b.json", "a.json", "a.wav", "notes.txt"] {
            fs::write(dir.join(name), "{}").expect("written");
        }
        let paths = document_paths(&dir).expect("the directory reads");
        let names: Vec<String> = paths
            .iter()
            .map(|path| path.file_name().expect("a name").to_string_lossy().into())
            .collect();
        assert_eq!(names, vec!["a.json".to_owned(), "b.json".to_owned()]);
        swept(&dir);
    }

    /// A file that will not read is carried, not fatal; a directory that will
    /// not read is fatal.
    #[test]
    fn an_unreadable_file_is_carried_and_an_unreadable_directory_is_not() {
        let dir = scratch("unreadable");
        fs::write(dir.join("good.json"), "{\"kind\": \"clip\"}").expect("written");
        // A directory named like a document: opening it as a file fails, which
        // is the error shape a caller reports as a skip.
        fs::create_dir_all(dir.join("bad.json")).expect("created");

        let read = documents(&dir).expect("the directory reads");
        assert_eq!(read.len(), 2);
        assert!(read[0].0.ends_with("bad.json"), "{:?}", read[0].0);
        assert!(read[0].1.is_err(), "a directory does not read as a file");
        assert_eq!(read[1].1.as_deref().expect("read"), "{\"kind\": \"clip\"}");

        assert!(documents(&dir.join("nowhere")).is_err());
        swept(&dir);
    }
}
