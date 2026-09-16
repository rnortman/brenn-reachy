//! The committed pose library, as the tests and checkers of this package read
//! it.
//!
//! `pose_library.textproto` is the asset every box binds by path, parsed on the
//! C++ side, so nothing in Rust sees the value a running box was configured
//! with. What is available is the bytes: this crate embeds the committed file
//! and reads it back through the same reader and the same screen a bound
//! library goes through, so a checker judging where the machine folded, and a
//! cog case handing a cog its configuration, both work off the file a unit
//! runs on rather than off a number restated beside it.
//!
//! The emitter's name sidecar comes with it, for the same reason and read
//! through the intent edge's own reader: a pose id is positional, so a reader
//! that wants the pose called `neutral` asks the sidecar rather than counting
//! documents.
//!
//! One crate rather than a copy per reader: two embeddings of one asset are two
//! answers able to disagree about which emit the tree holds.

use reachy_poses::config::{Library, parse_library, screen};
use reachy_poses::library::PoseLibrary;

/// The committed pose library, as the protobuf text a configuration dial takes.
const POSE_LIBRARY: &str = include_str!("pose_library.textproto");

/// The committed name-to-number sidecar the same emitter writes.
const LIBRARY_NAMES: &str = include_str!("library.names.json");

/// The committed library as the message a cog's dial hands a cog.
///
/// What a cog case sets on its wrapper's configuration, so the cog under test
/// reads the library the deployment ships.
///
/// # Panics
///
/// If the committed asset is not a library this build can read, which is a
/// broken emit rather than a case.
#[must_use]
pub fn message() -> &'static Library {
    static MESSAGE: std::sync::OnceLock<Library> = std::sync::OnceLock::new();
    MESSAGE.get_or_init(|| {
        parse_library(POSE_LIBRARY).expect("the committed pose library is the emitter's")
    })
}

/// The committed library as a message of the caller's own.
///
/// For a case that hands a cog a library differing from the committed one in
/// one field — a stow paced past a budget, most of all. Read afresh rather than
/// cloned from [`message`], so what a case alters is its own copy and the
/// shared one stays the deployment's.
///
/// # Panics
///
/// As [`message`].
#[must_use]
pub fn read() -> Library {
    parse_library(POSE_LIBRARY).expect("the committed pose library is the emitter's")
}

/// The same library, screened.
///
/// Where a reader outside a cog asks what a pose puts the machine at — the
/// stow most of all, which is what a release is judged against.
///
/// # Panics
///
/// If the committed asset does not screen, which is a broken emit: the emitter
/// screens what it writes.
#[must_use]
pub fn library() -> &'static PoseLibrary<'static> {
    static SCREENED: std::sync::OnceLock<PoseLibrary<'static>> = std::sync::OnceLock::new();
    SCREENED.get_or_init(|| {
        let bound = message()
            .validate()
            .expect("the committed pose library is a message this build reads");
        screen(bound).expect("the committed pose library screens")
    })
}

/// Where the committed library's stow puts every joint.
///
/// Solved once for the process, like the record a running session builds from
/// the same bytes: a checker that asked per judgement would solve a linkage per
/// row of a log.
///
/// # Panics
///
/// As [`library`], or if the committed stow does not solve.
#[must_use]
pub fn stow_joints() -> &'static reachy_motion::joints::JointVector {
    static FOLDED: std::sync::OnceLock<reachy_motion::joints::JointVector> =
        std::sync::OnceLock::new();
    FOLDED.get_or_init(|| {
        library()
            .stow_joints()
            .expect("the committed stow is a pose the linkage reaches")
    })
}

/// Both halves of the committed sidecar, as the intent edge reads them.
///
/// The edge's own reader rather than a walk of the JSON here: the sidecar is
/// one artifact with one shape, and a second reader of it is a second thing to
/// update when the emitter grows a field. Read once and handed out by
/// reference: a second read of one emit is a second opinion about which emit
/// this process holds, and every name lookup beside it would otherwise be a
/// fresh parse of the whole document.
///
/// # Panics
///
/// If the committed sidecar is not the emitter's JSON.
#[must_use]
pub fn tables() -> &'static (reachy_edge::MotionTable, reachy_edge::PoseTable) {
    static TABLES: std::sync::OnceLock<(reachy_edge::MotionTable, reachy_edge::PoseTable)> =
        std::sync::OnceLock::new();
    TABLES.get_or_init(|| {
        reachy_edge::parse(LIBRARY_NAMES).expect("the committed sidecar is the emitter's")
    })
}

/// What the sidecar says about the pose called `name`: its id and its pace.
///
/// # Panics
///
/// As [`tables`], or if the committed library carries no pose of that name.
#[must_use]
pub fn entry(name: &str) -> reachy_edge::PoseEntry {
    tables()
        .1
        .resolve(name)
        .unwrap_or_else(|| panic!("the committed pose library carries no pose named {name}"))
}

/// Where the pose called `name` puts the machine, as the committed library
/// states it.
///
/// # Panics
///
/// As [`entry`] and [`library`], or if the sidecar numbers a pose the library
/// does not hold — a skew between the two halves of one emit.
#[must_use]
pub fn targets(name: &str) -> reachy_motion::joints::JointTargets {
    let id = entry(name).pose_id;
    library()
        .targets(id)
        .unwrap_or_else(|| panic!("the committed library holds no pose {id}, which {name} names"))
        .0
}

#[cfg(test)]
mod tests {
    use super::{entry, library, message, targets};

    /// The committed asset reads, screens, and holds the stow every consumer
    /// resolves through it.
    #[test]
    fn the_committed_library_reads_and_screens() {
        let held = message()
            .validate()
            .expect("the committed pose library is a message this build reads");
        assert!(!held.poses.is_empty());
        let (stow_id, _, pace) = library().stow();
        assert_eq!(usize::from(stow_id), usize::from(held.stow));
        assert!(!pace.is_zero(), "a pace the screen admits is positive");
    }

    /// The two halves of one emit agree: the sidecar's stow is the library's,
    /// and a name resolves to the configuration the library holds under it.
    #[test]
    fn the_sidecar_numbers_the_library_this_crate_holds() {
        let stow = entry(reachy_poses::STOW_POSE);
        assert_eq!(stow.pose_id, library().stow().0);
        assert_eq!(targets(reachy_poses::STOW_POSE), library().stow().1);
        let neutral = targets(reachy_poses::NEUTRAL_POSE);
        assert_ne!(neutral, library().stow().1, "the fold is not the raise");
    }

    /// The geometric rules the two antenna poses are authored under, judged
    /// against the committed documents.
    ///
    /// Nothing between an authored pose and the machine expresses these: the
    /// document loader checks the envelope, which says nothing about which way
    /// an antenna leans, and the poses are operator-authorable. Each rule is a
    /// way an antenna pose can be wrong that no other check catches.
    #[test]
    fn the_antenna_poses_lean_the_way_the_motion_stack_reads_them() {
        use reachy_motion::phase::ANTENNA_CONTACT_BAND_RAD;
        use reachy_motion::tick::ANTENNA_OUTBOARD;

        let rest = targets(reachy_poses::NEUTRAL_POSE).antennas;
        let fold = targets(reachy_poses::STOW_POSE).antennas;
        for side in 0..2 {
            // A lean the wrong way is a lean into the other antenna's arc.
            assert!(
                rest[side] * ANTENNA_OUTBOARD[side] > 0.0,
                "the rest lean of antenna {side} is {}, which is not toward its own outboard \
                 direction {}",
                rest[side],
                ANTENNA_OUTBOARD[side],
            );
            // Resting outside the band would put the pair where no phase
            // judgement is made, so a pair that rests there is never judged.
            assert!(
                rest[side].abs() < ANTENNA_CONTACT_BAND_RAD,
                "the rest lean of antenna {side} is {} rad, outside the contact band of {} rad \
                 the phase check judges a pair in",
                rest[side],
                ANTENNA_CONTACT_BAND_RAD,
            );
            // Past straight down, on its own side: that is what loads the
            // gearbox play to one side and what makes the fold a fold.
            assert!(
                fold[side].abs() > core::f64::consts::PI,
                "antenna {side} folds to {} rad, which is short of straight down",
                fold[side],
            );
            assert!(
                fold[side] * ANTENNA_OUTBOARD[side] > 0.0,
                "antenna {side} folds to {} rad, which is past straight down on the other \
                 antenna's side",
                fold[side],
            );
        }
    }

    /// The figures `reachy-motion`'s stillness watch sizes its settle allowance
    /// on, held to the documents they were transcribed from.
    ///
    /// That crate is sans-I/O and reads no asset, so its widest-arc case states
    /// the fold and the rest as literals; this is the assertion that fails when
    /// a re-recorded `stow` or a hand-edited `neutral` moves out from under
    /// them. Without it the allowance can stop covering the travel with every
    /// test green, and the watch then reads a rod still arriving as one that
    /// will not stand still.
    #[test]
    fn the_antennas_are_where_the_stillness_allowance_was_sized_on() {
        assert_eq!(
            targets(reachy_poses::STOW_POSE).antennas,
            [-3.459_133_736_422_664_6, 3.359_641_315_834_277_7],
            "crates/reachy-motion/src/stillness.rs states this fold as FOLD",
        );
        assert_eq!(
            targets(reachy_poses::NEUTRAL_POSE).antennas,
            [-0.1745, 0.1745],
            "crates/reachy-motion/src/stillness.rs states this rest as REST",
        );
    }
}
