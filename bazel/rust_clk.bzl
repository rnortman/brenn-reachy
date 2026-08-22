"""This repo's `rust_clk_module`: rusty-cogs' macro with our two constants fixed.

The macro itself lives in rusty-cogs, which owns the generator it drives. Two of
its parameters are properties of this whole tree rather than of a module, and
this wrapper is where they are stated: `repo`, which heads every crate name the
generator derives -- a module compiled under another word fails at its importer,
not here -- and `validated_naming`, which every module of one importing web has
to agree on.

Everything else forwards untouched, and the wrapper keeps the label
`//bazel:rust_clk.bzl`, so a package that generates Rust loads what it always
did.
"""

load("@rusty_cogs//bazel:rust_clk.bzl", _rust_clk_module = "rust_clk_module")
load("//bazel:clk_naming.bzl", "VALIDATED_NAMING")

# The repository word every module in this tree is compiled under, reaching the
# generator as `--repo`. It has to be the Bazel module name this repo declares
# in `MODULE.bazel`: the word heads every crate name the generator derives, and
# a module in another repository importing one of ours spells that crate under
# the apparent repo name, which is the module name. Nothing checks the two
# agree. TODO(repo-word-agreement)
REPO = "brenn_reachy"

def rust_clk_module(
        name,
        src,
        repo = REPO,
        validated_naming = VALIDATED_NAMING,
        **kwargs):
    """Generates Rust for one `.clk` module under this repo's policy.

    Args:
        name: The generated crate's target, the module's stem plus `_clk_rs`.
        src: The `.clk` module, relative to the package, or a label when the
            module belongs to another repository and is generated in place.
        repo: The repository word the module is compiled under. Defaults to this
            tree's; a call names it only to generate a module that belongs to
            another repository, under that repository's word.
        validated_naming: Which surface holds each type's short name. Defaults
            to this tree's policy; a call names it only during a migration
            sweep, and every module of one importing web moves together. No call
            site passes it, so what holds the policy in force is hand-written
            Rust naming the wire types it produces -- `cogs/motion_slot_test.rs`
            and `cogs/proof/probe_cog_test.rs` spell `*Wire` types the other
            policy does not generate. Deleting those spellings deletes the only
            coverage this default has.
        **kwargs: Everything else, forwarded to `@rusty_cogs//bazel:rust_clk.bzl`'s
            `rust_clk_module` untouched -- see its docstring.
    """
    _rust_clk_module(
        name = name,
        src = src,
        repo = repo,
        validated_naming = validated_naming,
        **kwargs
    )
