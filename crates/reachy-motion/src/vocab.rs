//! What this layer adds to a vocabulary enum it does not own: the operator's
//! word for a variant, the variants that are not the schema's zero, and the
//! pin that holds a numbering still.
//!
//! A generated enum carries `Display` (the variant's name, a diagnostic
//! spelling), `VARIANTS` (every declared discriminant, the zero included) and
//! `Default` (the zero itself). What a report needs beyond that does not come
//! free: the prose an operator reads, and the list with the zero left out —
//! because the zero is the absence a slot holds, not a member of the set a sweep
//! means. The zero has a decode side too, since a field whose zero is the
//! unwritten-slot sentinel refuses it the same way it refuses a number outside
//! the vocabulary. All of it is stated once here rather than per consumer.

/// Every variant except the schema's zero.
///
/// For a vocabulary whose zero is the unwritten-slot sentinel: no register, no
/// failure, no shape. It is a declared discriminant, so `VARIANTS` includes it,
/// and every consumer that means "the things this vocabulary names" has to leave
/// it out. Which variant it is comes from the vocabulary's own `Default`, so a
/// renumbering cannot leave this filtering the wrong one, and a second sentinel
/// would be a change here rather than in nine call sites.
///
/// Not for a vocabulary whose zero carries meaning — `AuxStatus`, whose zero is
/// `ok`, is the standing counterexample: sweeping it through here would drop the
/// success case from whatever the sweep is checking.
pub fn without_zero<T, const N: usize>(variants: [T; N]) -> impl Iterator<Item = T>
where
    T: Copy + Default + PartialEq,
{
    variants
        .into_iter()
        .filter(|variant| *variant != T::default())
}

/// The variant a wire number names, unless it names the schema's zero.
///
/// The decode side of [`without_zero`], for a field whose zero is the
/// unwritten-slot sentinel: a number outside the vocabulary and the zero itself
/// answer the same way, because a slot nothing wrote and a slot holding a
/// number this build cannot name are both a slot to refuse. The caller keeps the
/// raw number for its error, which is the part worth reporting.
///
/// Takes the decode's own answer rather than the wire value: the generated wire
/// types share no trait, so `to_known` is called at the site and the zero is
/// filtered here.
///
/// Not for a vocabulary whose zero carries meaning — `AuxStatus`, whose zero is
/// `ok`, is the standing counterexample: passing it through here would refuse
/// the success case.
pub fn known_nonzero<T: Copy + Default + PartialEq>(known: Option<T>) -> Option<T> {
    known.filter(|variant| *variant != T::default())
}

/// Declare the test that pins a vocabulary's numbering to the numbers written
/// down here.
///
/// For a vocabulary whose numbers outlive the build that wrote them — a recorded
/// log, a slot one build writes and the next reads. The `match` is wildcard-free
/// over `VARIANTS`, so a variant added without a number does not compile and a
/// variant renumbered turns the test red; the number one past the end is
/// asserted to name nothing, which is the half that catches a decoder widened by
/// accident. Asserting the decoder against `VARIANTS` alone would restate the
/// generator's own definition of `to_known` and could not fail — the numbers
/// themselves are what a reader and a writer have in common, so the numbers are
/// what is written out.
#[macro_export]
macro_rules! vocab_numbering {
    (
        $(#[$doc:meta])*
        $name:ident: $ty:ident as $wire:ident, past the end $past:literal {
            $($variant:pat => $number:literal),+ $(,)?
        }
    ) => {
        $(#[$doc])*
        #[test]
        fn $name() {
            for variant in $ty::VARIANTS {
                let number = match variant {
                    $($variant => $number,)+
                };
                assert_eq!($wire::from(variant).0, number, "{variant:?}");
                assert_eq!($wire(number).to_known(), Some(variant), "and back");
            }
            assert_eq!($wire($past).to_known(), None, "past the end");
        }
    };
}

/// Declare a display adapter over a vocabulary enum: one word per variant, and
/// the test that they are distinct and say something.
///
/// An adapter rather than a `Display` impl because the enum belongs to a
/// generated crate, and this prose is this layer's opinion of a foreign type.
/// The `match` is wildcard-free, so a variant the vocabulary grows is a compile
/// error here rather than a report line that says nothing.
///
/// The test comes with the adapter instead of being written beside it: an
/// adapter's names are the whole content of a bring-up failure line, and a guard
/// that has to be remembered per adapter is a guard the next one ships without.
/// It lands as a module of a fixed name, so two adapters in one module collide
/// loudly rather than sharing one guard.
macro_rules! vocab_name {
    (
        $(#[$doc:meta])*
        $vis:vis struct $name:ident($ty:ty) {
            $($variant:pat => $word:literal),+ $(,)?
        }
    ) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        $vis struct $name(pub $ty);

        impl ::core::fmt::Display for $name {
            fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.write_str(match self.0 {
                    $($variant => $word,)+
                })
            }
        }

        #[cfg(test)]
        mod vocab_name_test {
            use super::*;

            /// Every variant has a word of its own, and a word that is more than
            /// a token. Two variants sharing one, or one rendering as nothing,
            /// would be invisible until the day a report mattered.
            #[test]
            fn every_name_is_distinct_and_says_something() {
                let mut seen = ::std::collections::BTreeSet::new();
                for variant in <$ty>::VARIANTS {
                    let word = $name(variant).to_string();
                    assert!(word.len() > 3, "{variant:?} renders as {word:?}");
                    assert!(seen.insert(word), "{variant:?} shares a name");
                }
                assert_eq!(seen.len(), <$ty>::VARIANTS.len());
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::{known_nonzero, without_zero};
    use crate::RegId;

    /// The zero and a number no variant carries answer alike, and everything
    /// else comes back whole.
    #[test]
    fn the_zero_decodes_as_no_variant_at_all() {
        assert_eq!(known_nonzero(Some(RegId::default())), None);
        assert_eq!(known_nonzero::<RegId>(None), None);
        assert_eq!(
            known_nonzero(Some(RegId::TorqueEnable)),
            Some(RegId::TorqueEnable)
        );
    }

    /// The filter removes exactly one variant, and the one the vocabulary's
    /// `Default` names. Asserted here rather than left to the consumers: their
    /// table counts would catch a regression, but report it as a table of the
    /// wrong length in another crate.
    #[test]
    fn the_zero_is_the_one_variant_left_out() {
        assert_eq!(
            without_zero(RegId::VARIANTS).count(),
            RegId::VARIANTS.len() - 1
        );
        assert!(!without_zero(RegId::VARIANTS).any(|reg| reg == RegId::default()));
        assert!(without_zero(RegId::VARIANTS).any(|reg| reg == RegId::TorqueEnable));
    }
}
