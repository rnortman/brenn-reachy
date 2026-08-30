//! A scratch directory a test can write into, removed when the test is done.
//!
//! Every crate here that has to hand a real path to code which reads a file
//! grew its own copy of the same four lines: a directory under the system
//! temporary directory named for the process, `create_dir_all`, the writes, and
//! a `remove_dir_all(..).ok()` at the end. The copies leave their directory
//! behind whenever the case fails — the cleanup is the last statement, and a
//! failed assertion never reaches it — and they reuse one name across runs,
//! so a leftover from a red run is what the next one reads.
//!
//! This is that idiom, once: the directory is unique per process *and* per
//! call, and the guard removes it on the way out of scope however the case
//! ended. There is no `tempfile` in this tree's crate universe and this is
//! smaller than adding one.
//!
//! Test support only. Nothing a unit runs links this.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

/// What makes two directories from one process different.
///
/// A process id alone is not enough: a case that makes two scratch directories
/// under one tag would have them be the same directory, and one case's cleanup
/// would take the other's files.
static NEXT: AtomicU64 = AtomicU64::new(0);

/// A directory that exists now and is gone when this value is dropped.
#[derive(Debug)]
pub struct Scratch {
    path: PathBuf,
}

/// A fresh, empty directory named after `tag`.
///
/// # Panics
///
/// If the directory cannot be made. A test that cannot write to the machine's
/// temporary directory is a test whose subject was never exercised, and saying
/// so here is what stops it from being read as a passing case.
#[must_use]
pub fn scratch_dir(tag: &str) -> Scratch {
    let nth = NEXT.fetch_add(1, Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!("{tag}-{}-{nth}", std::process::id()));
    // Whatever a previous run left at this name is not this run's: a pid can be
    // reused, and a directory holding another run's files would be read as this
    // one's.
    std::fs::remove_dir_all(&path).ok();
    std::fs::create_dir_all(&path).expect("a scratch directory this test can write into");
    Scratch { path }
}

impl Scratch {
    /// A path inside the directory. Nothing is created by asking.
    #[must_use]
    pub fn join(&self, name: &str) -> PathBuf {
        self.path.join(name)
    }
}

impl AsRef<Path> for Scratch {
    fn as_ref(&self) -> &Path {
        &self.path
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        // Best effort, and deliberately not asserted: a case that already
        // failed should report what it found rather than how the cleanup went.
        std::fs::remove_dir_all(&self.path).ok();
    }
}

#[cfg(test)]
mod tests {
    use super::scratch_dir;

    #[test]
    fn a_scratch_directory_is_there_and_then_is_not() {
        let path;
        {
            let dir = scratch_dir("reachy-scratch-case");
            path = dir.join("a-file");
            std::fs::write(&path, "x").expect("a file");
            assert!(path.exists(), "the file the case wrote is there");
        }
        assert!(!path.exists(), "the directory went with the guard");
    }

    #[test]
    fn two_directories_under_one_tag_are_two_directories() {
        let first = scratch_dir("reachy-scratch-case");
        let second = scratch_dir("reachy-scratch-case");
        assert_ne!(first.join("f"), second.join("f"));
    }
}
