//! Deterministic pseudo-random source shared by this crate's tests.
//!
//! One seeded generator for every randomised test here — the sweeps over
//! random poses that pin the solvers need thousands of samples, and a failure
//! has to reproduce exactly from the seed printed in the test. Kept in one
//! place so the per-module copies cannot drift into different distributions.

/// xorshift64. Small, fast, and good enough for sampling poses; nothing here
/// is cryptographic.
pub(crate) struct Rng(u64);

impl Rng {
    /// Seeded generator. A zero seed is degenerate for xorshift, so it is
    /// refused rather than silently producing an all-zero stream.
    pub(crate) fn new(seed: u64) -> Self {
        assert!(seed != 0, "xorshift needs a nonzero seed");
        Self(seed)
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in `[lo, hi)`, from the top 53 bits so every sample is an
    /// exactly representable multiple of 2⁻⁵³ of the span.
    pub(crate) fn range(&mut self, lo: f64, hi: f64) -> f64 {
        let unit = (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
        lo + unit * (hi - lo)
    }
}
