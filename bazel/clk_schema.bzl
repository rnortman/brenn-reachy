"""A schema-only `.clk` module's whole build, as one macro.

Every module in `geometry/`, `motion/` and `hardware/dynamixel/` is the same
shape: no cogs, no proto backend, a Rust crate for the logic layer, and a C++
half because the cog modules that import it generate C++ too and a generated C++
type refers to its imports' generated C++ types. Written out, that is two rule
stanzas whose `outs` are three mechanically derived filenames and whose
dependency edges are one list of modules spelled four different ways -- for the
C++ libraries, for the `.clk` compile, for the generator's import path, and for
the Rust crates. A dozen such modules exist and the design adds more.

So the shape is stated once here and each module is one call naming what it
imports. A dropped `cpp_deps` entry or a mistyped `outs` name is no longer
possible to write, and the four spellings cannot drift apart because there is
only one list.

A module needing anything else -- the proto backend, a cog, a generated
extern-"C" archive -- is not this shape and spells its own stanzas, which is what
`cogs/BUILD.bazel` does.
"""

load("@clockwork//clockwork:rules.bzl", "clk")
load("//bazel:clk_naming.bzl", "VALIDATED_NAMING")
load("//bazel:rust_clk.bzl", "rust_clk_module")

def _module_label(module, suffix):
    """One module dependency, spelled for one of the four lists.

    Args:
        module: A sibling module's stem (`"joints"`), or another package's module
            as a label (`"//hardware/dynamixel:registers"`).
        suffix: What the target or file this list wants is called relative to the
            module: `"_clk"`, `"_clk_cc"`, `"_clk_rs"` or `".clk"`.

    Returns:
        The label. A sibling is `":joints_clk"` for a target list and bare
        `"joints.clk"` for the generator's imports, which is the one place a
        same-package module is named as a file rather than as a target.
    """
    if module.startswith("//") or module.startswith("@"):
        return module + suffix
    if suffix == ".clk":
        return module + suffix
    return ":" + module + suffix

def _labels(modules, suffix):
    """The dependency list for one of the four spellings, sorted."""
    return sorted([_module_label(module, suffix) for module in modules])

def clk_module_deps(modules):
    """One module list, in the four spellings a dependent target names it in.

    A module is a dependency four times over: the C++ library its generated C++
    refers to, the `.clk` compile, the generator's import path, and the Rust
    crate. A package that spells the four lists out by hand can drop an entry
    from one of them, and the build then fails somewhere that does not name the
    missing edge. So the list is written once and expanded here.

    `clk_schema_module` does this for a schema-only module's own two stanzas;
    this is the same expansion for a package that spells its stanzas itself,
    which is what `cogs/BUILD.bazel` does for the cog and system modules.

    Args:
        modules: The modules a target's `.clk` reaches, transitively. Sibling
            modules by stem, other packages' by label.

    Returns:
        A struct with the four sorted lists: `cpp` for `cpp_deps`, `clk` for a
        `clk()` rule's `deps`, `imports` for the generator's import paths, and
        `rs` for a `rust_clk_module`'s `deps`.
    """
    return struct(
        cpp = _labels(modules, "_clk_cc"),
        clk = _labels(modules, "_clk"),
        imports = _labels(modules, ".clk"),
        rs = _labels(modules, "_clk_rs"),
    )

def clk_schema_module(name, imports = []):
    """A schema-only module: its C++ half, its Rust crate, and its export.

    Args:
        name: The module's stem. The source is `<name>.clk` and the targets are
            `<name>_clk` (C++) and `<name>_clk_rs` (Rust), which is what an
            importing package names.
        imports: The modules this one's `use` lines name -- transitively, because
            a `.clk` compile has to resolve every module reachable from the one
            it is given, not just the directly referenced ones. Sibling modules
            by stem, other packages' by label.
    """
    src = name + ".clk"
    native.exports_files([src])

    clk(
        name = name + "_clk",
        srcs = [src],
        outs = [
            name + "_clk_cc.cc",
            name + "_clk_cc.hh",
            name + "_clk_cc.inl",
        ],
        cpp_deps = _labels(imports, "_clk_cc"),
        generate = ["cpp"],
        visibility = ["//visibility:public"],
        deps = _labels(imports, "_clk"),
    )

    rust_clk_module(
        name = name + "_clk_rs",
        src = src,
        imports = _labels(imports, ".clk"),
        validated_naming = VALIDATED_NAMING,
        deps = _labels(imports, "_clk_rs"),
    )
