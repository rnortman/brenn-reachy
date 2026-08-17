"""The macro the pinned clang sysroot archive expects to find in the root repo."""

# The sysroot archive's own BUILD file loads `@@//tools/sysroot:sysroot.bzl` --
# an absolute reference to the *main* repository -- and calls `make_sysroot`. So
# this file's path and this symbol's name are both fixed by the archive, and
# every consumer of the drop has to supply them. The body is a re-export: the
# real macro, and the toolchain configuration under it, stay in the drop.

load("@clockwork//tools/sysroot:sysroot.bzl", _make_sysroot = "make_sysroot")

make_sysroot = _make_sysroot
