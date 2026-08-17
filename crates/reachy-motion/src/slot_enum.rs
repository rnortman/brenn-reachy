//! One declaration for the enums that cross a fixed-layout slot.
//!
//! A host that keeps this crate's state in a slot holds every enum in it as an
//! integer, so each such type owes the same four things: a stated numbering, the
//! list of what exists, the way out to an integer and the way back. Written by
//! hand that is forty lines a type, the variant list appears twice — once as the
//! enum body and once as `ALL` — and nothing holds the two together: a variant
//! added to the body and forgotten in `ALL` compiles, gets a discriminant, is
//! written into a slot, and then refuses to come back out.
//!
//! [`slot_enum`] takes the list once and emits both, so that failure is not
//! expressible. What it does not do is choose the numbering: every value is
//! written out at its variant, because the numbers are the part a slot written
//! by one build and read by the next depends on.

/// An enum with a stated integer numbering, its list, and a refusing decoder.
///
/// The body is the variant list with each variant's number written out. The
/// three header lines name the two accessors (a signed numbering wants
/// `as_i8`/`from_i8`, not `as_u8`) and the sentence the decoder's documentation
/// ends with, which says what a number outside the list means for that
/// particular type.
///
/// ```ignore
/// slot_enum! {
///     /// Which way a pose solve failed, as a number.
///     pub enum FkFailureCode: u8 {
///         encode: as_u8;
///         decode: from_u8;
///         refusal: "A number outside the three names no failure this build knows.";
///
///         /// The fault is not about a pose solve.
///         NotApplicable = 0,
///         /// [`FkError::NoConvergence`]: iterations, then the largest residual.
///         NoConvergence = 1,
///     }
/// }
/// ```
macro_rules! slot_enum {
    // One `()` per variant, so the list can be counted where a length is
    // wanted. `${count(...)}` would say this directly and is not stable on the
    // toolchain this crate builds under.
    (@unit $anything:ident) => { () };
    (
        $(#[$enum_meta:meta])*
        $vis:vis enum $name:ident: $int:ident {
            encode: $as_fn:ident;
            decode: $from_fn:ident;
            refusal: $refusal:expr;

            $(
                $(#[$variant_meta:meta])*
                $variant:ident = $value:expr
            ),+ $(,)?
        }
    ) => {
        $(#[$enum_meta])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        #[repr($int)]
        $vis enum $name {
            $(
                $(#[$variant_meta])*
                $variant = $value,
            )+
        }

        impl $name {
            /// Every one of them, in numbering order.
            ///
            /// Emitted from the same list as the variants, so it cannot name
            /// fewer of them than exist.
            pub const ALL: [$name; <[()]>::len(&[$(slot_enum!(@unit $variant)),+])] =
                [$($name::$variant),+];

            #[doc = concat!(
                "This as the ", stringify!($int), " a slot holds it in.\n\n",
                "The numbering is part of the API: a number written by one \
                 build reads as the same thing in the next, so values are \
                 appended and never renumbered."
            )]
            #[must_use]
            pub fn $as_fn(self) -> $int {
                self as $int
            }

            #[doc = concat!(
                "What `value` names, or `None` if it names none.\n\n",
                $refusal
            )]
            #[must_use]
            pub fn $from_fn(value: $int) -> Option<Self> {
                Self::ALL.into_iter().find(|it| it.$as_fn() == value)
            }
        }
    };
}

pub(crate) use slot_enum;
