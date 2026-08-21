"""The validated-naming policy every `.clk` module in this repo is generated under.

One constant rather than a literal per target, because the policy is a property
of the whole dependency web and not of a module: a module re-derives an imported
type's name under its *own* policy, so a web split across policies fails at
rustc in the consumer, naming an identifier that exists nowhere. Written once,
`load()`ed by every package that generates Rust from a `.clk`, and read by
`rust_clk_module`'s default -- so a module that names no policy is born on the
web's, and a sweep to the next policy is this line.

`suffixed` leaves the short name on the open wire types; `wire-suffixed` is the
migration step where both surfaces are suffixed and no type holds the plain
name; `flipped` is the end state, the validated types holding the short name and
the open ones suffixed `Wire`, and is what this web generates under.
"""

VALIDATED_NAMING = "flipped"
