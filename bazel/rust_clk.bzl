"""The Rust half of a `.clk` module's build, as one macro.

A copy of rusty-cogs' `bazel/rust_clk.bzl` at rev 4e8f8e8, the revision
MODULE.bazel pins. It is copied rather than `load()`ed because `REPO` below is a
constant of the file and heads every crate name the generator derives: loading
theirs would compile this repo's modules under crate names spelling their
repository, which no importer here writes. On a pin bump, re-copy this file and
re-apply the four label and constant changes named below.

TODO(rusty-cogs-macro-parameters)

Upstream's `clk()` owns the C++ side of a module: it runs the compiler and
declares the libraries over what comes out. This is the same thing for the Rust
side -- the generation action, the crate the generated types live in, and, for a
cog-bearing module, the archive of `extern "C"` entry points the generated C++
shim links against. A package that generates Rust calls this once per module and
spells none of that shape itself.

Two crates rather than one for a cog module: the entry points have to sit in a
separately linkable archive the generated C++ shim links against, beside a dial
crate the author's cog code can depend on without pulling the FFI in.
"""

load("@rules_rust//rust:defs.bzl", "rust_library", "rust_static_library")

# What the C++ standard library is called on the link line of anything that pulls
# the test shim in. A `rust_test` is linked by rustc, which passes
# `-nodefaultlibs` and so names no C++ runtime; the shim's own translation unit
# and the whole upstream harness behind it need one, and a `cc_library`'s
# `linkopts` reach the Rust binary through `CcInfo`. Nothing on the execute path
# needs this: there the C++ toolchain drives the link.
#
# The literal assumes a GNU toolchain, which is what this repo builds and tests
# under. Under clang with libc++ the link fails on hundreds of undefined `std::`
# symbols and this line is what has to change -- to `-lc++`, or to a `select()`
# on the compiler once more than one is built here.
_CPP_STDLIB_LINKOPTS = ["-lstdc++"]

# The repository name every module in this tree is compiled under. It reaches the
# generator as `--repo` and heads every crate name the generator derives, so the
# two have to be one word -- a crate built under a name no importer spells fails
# at the importer rather than here.
REPO = "brenn_reachy"

# The edition generated Rust is written for. It is the emitter's contract rather
# than this build's, so it is stated here, beside the rules that compile what the
# emitter wrote, and nowhere else on the Bazel side.
GENERATED_EDITION = "2024"

# What a generated crate's target is called: the module's stem plus this. The
# suffix is load-bearing beyond taste: `.gitignore` covers `*_clk_rs.rs` and
# `*_clk_cc_impl.cc` as build artifacts, so a differently named output would be
# committed by accident.
_TARGET_SUFFIX = "_clk_rs"

_CLK_EXT = ".clk"

# The runtime crate, which declares the message types and carries the only
# `unsafe` the generated code rests on. Every generated crate depends on it and,
# apart from its sibling modules, on nothing else -- which is what keeps the
# coupling to a compiler drop inside the generator rather than in what the
# generator writes. It comes from rusty-cogs, not from here.
_RUNTIME = "@rusty_cogs//crates/clockwork-rs:clockwork_rs"

# The framework's own `.clk` modules, needed in the sandbox of any cog-bearing
# compile: the compiler injects telemetry imports into a cog with metrics
# enabled, which is the default, so a compile that cannot resolve them fails
# inside the upstream importer.
_FRAMEWORK_IMPORTS = "//bazel:framework_clk_imports"

_GENERATOR = "@rusty_cogs//generator:rusty_cogs_gen"

# The toolchain's own formatter, which the generator writes every `.rs` artifact
# through. It arrives as a build tool so that the version formatting the output
# is the version the gate's format lanes check it with.
_RUSTFMT = "@rules_rust//rust/toolchain:current_rustfmt_toolchain"


def _crate_name(package, target_name, src_dir):
    """The crate name the generator derives for a module.

    Args:
        package: The package the module is declared in, which is
            `native.package_name()` at every call site and a case at the test
            below. It is a parameter rather than a lookup so that the derivation
            is a pure function of its inputs and can be checked without a
            package to stand in.
        target_name: The generated crate's target name.
        src_dir: The module source's directory, relative to the package.

    Returns:
        The repository and the module's path segments joined with doubled
        underscores, which is what the generator writes and therefore what
        another generated crate importing this module spells. A target name is
        unique within its package; a crate name has to be unique across every
        module every repository generates, which is why it is derived from the
        path rather than defaulted to the target name.

        The generator owns this rule; this is the build's restatement of it, and
        the generation action is passed the result so the generator can refuse a
        disagreement. Without that the two would be held together by convention,
        and a schema-only module's divergence would surface packages away, at
        whichever compile first failed to find the crate it imports.
    """
    path = package
    if src_dir:
        path = path + "/" + src_dir if path else src_dir
    segments = [REPO] + [segment for segment in path.split("/") if segment]
    return "__".join(segments + [target_name])


def _module_error(name, src):
    """Why this target name and this source cannot belong to one module.

    Every output name the macro declares is derived from the target name, and
    `.gitignore` keys off that spelling, so a module whose artifacts are named
    after another one is a committed generated file or a crate nobody can find.
    The names are held to one rule here instead.

    Args:
        name: The generated crate's target name.
        src: The `.clk` module, relative to the package.

    Returns:
        The message the macro refuses with, or `None` when the two agree.
    """
    if not name.endswith(_TARGET_SUFFIX):
        return "rust_clk_module: name '%s' must end in '%s'" % (name, _TARGET_SUFFIX)
    file_name = src.rpartition("/")[2]
    if not file_name.endswith(_CLK_EXT):
        return "rust_clk_module: src '%s' is not a '%s' module" % (src, _CLK_EXT)
    stem = file_name[:-len(_CLK_EXT)]
    if name != stem + _TARGET_SUFFIX:
        return "rust_clk_module: src '%s' wants the name '%s', not '%s'" % (
            src,
            stem + _TARGET_SUFFIX,
            name,
        )
    return None


def _test_wrappers_error(name, cog_impl):
    """Why this module cannot have unit-test wrappers generated for it.

    The wrappers are a Rust surface over upstream's harness driving *this
    module's* cogs through their generated entry points. A module with no Rust
    cog implementation has no entry points, so the crate would compile against a
    facade nothing of ours reaches -- true but useless, and a testonly target
    that quietly tests nothing is worse than a refusal.

    Args:
        name: The generated crate's target name.
        cog_impl: The crate defining each cog's body, or `None`.

    Returns:
        The message the macro refuses with, or `None` when the two agree.
    """
    if not cog_impl:
        return "rust_clk_module: '%s' sets test_wrappers without cog_impl" % name
    return None

def rust_clk_module(
        name,
        src,
        imports = [],
        deps = [],
        cog_impl = None,
        test_wrappers = False,
        visibility = ["//visibility:public"]):
    """Generates Rust for one `.clk` module and declares the crates over it.

    Args:
        name: The generated crate's target, which must be the module's stem plus
            `_clk_rs`. Every other target and output name is derived from it, so
            that one module's artifacts cannot be named after another's.
        src: The `.clk` module, relative to this package.
        imports: Further `.clk` sources the compile needs in its sandbox. The
            generator compiles in process through an importer that reads source
            from disk, so a module this one `use`s has to be an input of the
            action or the compile fails module-not-found. The framework's own
            modules arrive on their own for a cog module.
        deps: Generated crates this module's code refers to -- one per module it
            imports a declared type from. The runtime crate is always a dep and
            is not listed here.
        cog_impl: The crate defining each cog's body, for a cog-bearing module.
            Its presence is what makes this a cog compile: the entry points and
            the C++ shim are emitted, and an alias binds this label to the fixed
            name the generated code spells, so the generator needs no knowledge
            of what the author called the crate.
        test_wrappers: Whether to generate the Rust unit-test wrappers over
            upstream's own cog test harness, as the testonly crate
            `<name>_test`. Requires `cog_impl`, since a wrapper around no Rust
            cog tests nothing. It requires one thing of the caller that this
            macro cannot declare for it: the module's `clk()` target must list
            `cpp_test_cog` in `generate` and carry the three
            `<stem>_cc_test.{cc,hh,inl}` outs, because the wrapper is a C ABI
            over the facade that generation emits. Without them the missing
            `:<stem>_cc_test` target fails analysis before anything compiles.
        visibility: Visibility of the generated crate. The entry archive is not
            published: it is linked by the shim's `cc_library` in this package
            and means nothing anywhere else. The test crate follows this
            visibility, so a test in another package can drive the wrappers.
    """
    error = _module_error(name, src)
    if error != None:
        fail(error)

    if test_wrappers:
        error = _test_wrappers_error(name, cog_impl)
        if error != None:
            fail(error)

    gen = name + "_gen"
    dial_file = name + ".rs"
    entry_file = name + "_entry.rs"
    test_file = name + "_test.rs"
    crate = _crate_name(native.package_name(), name, src.rpartition("/")[0])

    # The `clk()` target's name, which is this target's without its `_rs`. Every
    # C++ artifact of the module -- upstream's and ours -- is named after it.
    clk_target = name[:-len("_rs")]
    test_shim = clk_target + "_cc_test_shim"

    gen_srcs = [src] + imports
    outs = [dial_file]
    args = [
        "$(location %s) generate" % _GENERATOR,
        "--input $(location %s)" % src,
        "--output $(location %s)" % dial_file,
    ]

    if cog_impl:
        # The shim must be named `<clk target>_cc_impl.cc`: `cpp_cog`
        # generates a dependency on that name. The `clk()` target is the
        # module's stem plus `_clk`, which is this target's name without
        # its `_rs`.
        shim_file = clk_target + "_cc_impl.cc"
        gen_srcs = gen_srcs + [_FRAMEWORK_IMPORTS]
        outs = outs + [entry_file, shim_file]
        args = args + [
            "--entry-output $(location %s)" % entry_file,
            "--shim-output $(location %s)" % shim_file,
        ]

    if test_wrappers:
        # Named after the `clk()` target like the execute shim, with `_shim` to
        # keep it clear of upstream's own `<clk target>_cc_test.cc` -- the facade
        # this one is a C ABI over, generated into the same package.
        test_shim_file = test_shim + ".cc"
        outs = outs + [test_file, test_shim_file]
        args = args + [
            "--test-output $(location %s)" % test_file,
            "--test-shim-output $(location %s)" % test_shim_file,
        ]

    args = args + [
        "--repo %s" % REPO,
        # What the libraries below are declared as. The generator derives the
        # same name from the module and fails the action if the two disagree.
        "--crate-name %s" % crate,
        # The execution root, which is where a Bazel action's paths are relative
        # to and what the upstream compiler's own path resolution assumes.
        "--root .",
        "--rustfmt $(location %s)" % _RUSTFMT,
    ]

    # One action for every artifact of a module: they only mean anything together
    # -- a dial with no entry point is unreachable, an entry point with no shim is
    # unreferenced -- so the generator writes all of them or none.
    native.genrule(
        name = gen,
        srcs = gen_srcs,
        outs = outs,
        cmd = " ".join(args),
        tools = [
            _GENERATOR,
            _RUSTFMT,
        ],
    )

    rust_library(
        name = name,
        srcs = [":" + gen],
        crate_name = crate,
        edition = GENERATED_EDITION,
        visibility = visibility,
        deps = [_RUNTIME] + deps,
    )

    if cog_impl:
        rust_static_library(
            name = name + "_entry",
            srcs = [":" + gen],
            aliases = {cog_impl: "cog_impl"},
            crate_name = crate + "_entry",
            crate_root = entry_file,
            edition = GENERATED_EDITION,
            deps = [
                ":" + name,
                cog_impl,
                _RUNTIME,
            ],
        )

    if test_wrappers:
        # `:<clk target>_cc_test` is upstream's generated facade over its
        # templated harness, and comes from the module's own `clk()` rather than
        # from here. It is testonly there, so everything over it is testonly too
        # -- the harness stands up shared memory and a temporary directory per
        # instance and must never reach a shipping binary.
        native.cc_library(
            name = test_shim,
            srcs = [test_shim_file],
            linkopts = _CPP_STDLIB_LINKOPTS,
            testonly = True,
            deps = [":" + clk_target + "_cc_test"],
        )

        # One dep for a consumer's `rust_test`: the C ABI rides along with the
        # crate that declares it, so a test target names this and the crates
        # whose message types its assertions spell. It rides in `link_deps`
        # rather than `deps` -- the rules reserve `deps` for Rust crates and
        # `link_deps` is where a native archive linked by hand-written `extern
        # "C"` declarations belongs.
        rust_library(
            name = name + "_test",
            srcs = [":" + gen],
            crate_name = crate + "_test",
            crate_root = test_file,
            edition = GENERATED_EDITION,
            link_deps = [":" + test_shim],
            testonly = True,
            visibility = visibility,
            deps = [
                ":" + name,
                _RUNTIME,
            ] + deps,
        )
