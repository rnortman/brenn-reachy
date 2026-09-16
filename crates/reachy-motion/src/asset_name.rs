//! What an asset may be called, for every kind of asset there is.
//!
//! A clip, a motion and a pose all live in one namespace addressed by one wire
//! field, so one rule governs all three. It sits here rather than in either
//! asset crate because `reachy-clips` and `reachy-poses` are peers: neither may
//! be the other's authority, and a rule stated twice is a rule that can drift.
//!
//! `motion-proto` remains the authority on the *length* bound. It has no
//! dependencies and cannot take one — it is published across a repository seam
//! — so its copy is the contract and the mirror below is held to it by the
//! drift guard in `cogs/edge_caps_test`, the one target that links both. The
//! charset and path shape are this side's alone: the wire carries a name, and
//! what makes a name usable as a file stem is the library's business.

use thiserror::Error;

/// The longest an asset name may be, characters.
///
/// Names are the join key between the wire and a library, so they travel in
/// every script that invokes an asset; a bound keeps a script's size a function
/// of its step count.
///
/// Must equal `motion_proto::MAX_ASSET_NAME_LEN`, which is authoritative. The
/// two crates share no dependency, so `cogs/edge_caps_test` — which links both
/// — is the only enforcement, and it holds this copy to that one.
pub const MAX_ASSET_NAME_LEN: usize = 128;

/// Why a name cannot be used.
///
/// Names reach a script, a report line and a file stem, so the charset is
/// narrow and the refusal says which character offended rather than restating
/// the rule.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum AssetNameError {
    /// The name is the empty string.
    #[error("an asset name may not be empty")]
    Empty,

    /// The name is longer than [`MAX_ASSET_NAME_LEN`].
    #[error("an asset name may be at most {MAX_ASSET_NAME_LEN} characters; this one is {len}")]
    TooLong {
        /// The name's length in characters.
        len: usize,
    },

    /// A character outside `[a-z0-9_./-]`.
    #[error("an asset name may only hold [a-z0-9_./-]; this one holds {ch:?}")]
    BadChar {
        /// The first offending character.
        ch: char,
    },

    /// A leading `/`, a trailing `/`, or a `//`.
    #[error("an asset name may not hold an empty path segment")]
    EmptySegment,

    /// A `.` or `..` segment.
    #[error("an asset name may not hold a \".\" or \"..\" segment")]
    DotSegment,

    /// A leading `-`.
    #[error("an asset name may not begin with \"-\"")]
    LeadingDash,
}

/// Check an asset name against the charset, the length bound, and the shape a
/// relative path may take.
///
/// Every asset lives in one namespace, addressed by one wire field, so one
/// rule. The path shape belongs to that rule rather than to each consumer,
/// because a name *becomes* a path: the clip importer writes a clip and its
/// audio sidecar under it, a pose document's file stem is its name, and a name
/// a consumer joins onto a directory is the whole of what stops a downloaded,
/// converted document from writing outside it. So a name is a relative path
/// with no navigation in it — no leading slash, no empty segment, no `.` or
/// `..` — and does not open with a `-`, which reads as an option wherever a
/// name reaches a command line.
///
/// # Errors
///
/// [`AssetNameError`], naming the first rule the name breaks.
pub fn check_asset_name(name: &str) -> Result<(), AssetNameError> {
    if name.is_empty() {
        return Err(AssetNameError::Empty);
    }
    let len = name.chars().count();
    if len > MAX_ASSET_NAME_LEN {
        return Err(AssetNameError::TooLong { len });
    }
    if let Some(ch) = name
        .chars()
        .find(|ch| !matches!(ch, 'a'..='z' | '0'..='9' | '_' | '.' | '/' | '-'))
    {
        return Err(AssetNameError::BadChar { ch });
    }
    if name.starts_with('-') {
        return Err(AssetNameError::LeadingDash);
    }
    if name.split('/').any(str::is_empty) {
        return Err(AssetNameError::EmptySegment);
    }
    if name
        .split('/')
        .any(|segment| segment == "." || segment == "..")
    {
        return Err(AssetNameError::DotSegment);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{AssetNameError, MAX_ASSET_NAME_LEN, check_asset_name};

    /// The charset and the length bound, including both ends of the bound: a
    /// name at the cap is usable and one character past it is not.
    #[test]
    fn a_name_is_lowercase_and_bounded() {
        assert_eq!(check_asset_name("pod/nod-twice_2.v1"), Ok(()));
        assert_eq!(check_asset_name(""), Err(AssetNameError::Empty));
        assert_eq!(
            check_asset_name("Pollen/x"),
            Err(AssetNameError::BadChar { ch: 'P' })
        );
        assert_eq!(
            check_asset_name("a b"),
            Err(AssetNameError::BadChar { ch: ' ' })
        );
        let long = "a".repeat(MAX_ASSET_NAME_LEN + 1);
        assert_eq!(
            check_asset_name(&long),
            Err(AssetNameError::TooLong {
                len: MAX_ASSET_NAME_LEN + 1
            })
        );
        assert_eq!(check_asset_name(&"a".repeat(MAX_ASSET_NAME_LEN)), Ok(()));
    }

    /// A name becomes a path, so the rule is the one that makes joining it onto
    /// a directory safe: nothing absolute, nothing empty, no navigation, and
    /// nothing that reads as a command-line option.
    #[test]
    fn a_name_is_a_relative_path_with_no_navigation_in_it() {
        assert_eq!(
            check_asset_name("/etc/cron.d/x"),
            Err(AssetNameError::EmptySegment)
        );
        assert_eq!(
            check_asset_name("pollen//x"),
            Err(AssetNameError::EmptySegment)
        );
        assert_eq!(
            check_asset_name("pollen/"),
            Err(AssetNameError::EmptySegment)
        );
        assert_eq!(
            check_asset_name("../../persistent/x"),
            Err(AssetNameError::DotSegment)
        );
        assert_eq!(
            check_asset_name("pollen/../../x"),
            Err(AssetNameError::DotSegment)
        );
        assert_eq!(check_asset_name("."), Err(AssetNameError::DotSegment));
        assert_eq!(check_asset_name(".."), Err(AssetNameError::DotSegment));
        assert_eq!(
            check_asset_name("--force"),
            Err(AssetNameError::LeadingDash)
        );
        assert_eq!(check_asset_name("-x"), Err(AssetNameError::LeadingDash));

        // Dots inside a segment are ordinary characters; only a whole segment
        // that is `.` or `..` navigates.
        assert_eq!(check_asset_name("pollen/emotions/loving1.v2"), Ok(()));
        assert_eq!(check_asset_name("...."), Ok(()));
    }
}
