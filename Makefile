# Repo-root Makefile — the check gate, the tree sweep, and hook wiring.
#
# The targets hold the contents of the gate, so both the pre-commit hook and CI
# invoke one definition rather than two copies that drift, and the whole gate is
# reproducible on a fresh clone. `check` is what a commit must pass and what CI
# runs, one lane over the whole tree. The secret sweep is a separate target, and
# separate in CI too: it scans content, not correctness, and answers a different
# question.

# Recipes run under bash with -e and pipefail, so a failing command anywhere in a
# chain or a pipeline fails the target. Without this a broken step degrades into a
# green result, which is the worst thing a gate can do.
SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Repo-root targets:"
	@echo "  make check         what a commit must pass: shellcheck, script tests, bazel test"
	@echo "  make check-device  the other half of the gate: the device binaries cross-compile"
	@echo "  make fix           auto-fix what the gate can fix (rustfmt)"
	@echo "  make setup-hooks   wire git at .githooks, check tooling (once per clone)"
	@echo "  make scrub-tree    whole-tree secret sweep — the sweep a clean tree is declared on"
	@echo "  make clip-config   regenerate the clip library asset from cogs/clips/"
	@echo "  make motion-host-run  run the online system here, against the simulated plant"
	@echo ""
	@echo "Device targets — real hardware, no part of any gate. Need bazel, and a"
	@echo "REACHY_HOST naming a reachable unit:"
	@echo "  make bench-build     the aarch64 bench binary, cross-compiled by bazel"
	@echo "  make bench-config    push the bench's configuration into the unit's RAM"
	@echo "  make bench-run       build, push, run the bench on the unit (ARGS=...)"
	@echo "  make bench-selftest  build, push, run the read-only registry on the unit"
	@echo "  make bench-fetch     bring a run's state file back, timestamped"
	@echo "  make motion-build    the aarch64 motion payload: launcher, binaries, configs"
	@echo "  make motion-deploy   build and push the payload into the unit's RAM"
	@echo "  make motion-run      build, push, run on the unit, fetch and judge the log"
	@echo "  make motion-fetch    bring a run's .olog directories back, timestamped"

# The shell half of the gate. These scripts push binaries and configuration onto
# real hardware and drop privileges there, and they already carry
# `# shellcheck source=` directives, which mean nothing unless something runs
# them.
#
# The file set is every `.sh` in the working tree — tracked, plus the ones not
# added yet, with the ignore rules honoured — plus the two extensionless git
# hooks, which have no suffix to be found by. Tracked-plus-untracked because
# "the gate is green" has to be a statement about the tree in front of you: a
# script written and not yet staged is a script the next commit carries, and a
# walk of the index alone reports green without ever having read it.
#
# The flags are load-bearing: -x follows the sourced files, and -P SCRIPTDIR
# resolves a `source=` directive against the directory of the script carrying it
# rather than the working directory — which is what the directives assume, and
# without which a clean tree reports SC1091 and SC2154 findings that are
# artifacts of the invocation.
#
# The set arrives NUL-separated through xargs rather than as an unquoted
# substitution: a path carrying a space would otherwise split into arguments
# naming nothing, and one starting with a dash would be read as a flag. `--`
# ends the options on both invocations for the same reason.
#
# Hard-required, never skipped when the tool is absent: a gate that passes on a
# machine missing shellcheck and fails on CI inverts the local-gate-first rule.
.PHONY: check-scripts
check-scripts:
	@command -v shellcheck >/dev/null 2>&1 || { \
	    echo "shellcheck not found on PATH — the script half of the gate cannot run." >&2; \
	    echo "Install it from your distribution (Fedora: dnf install ShellCheck;" >&2; \
	    echo "Debian and Ubuntu: apt install shellcheck), or from" >&2; \
	    echo "https://github.com/koalaman/shellcheck#installing." >&2; \
	    exit 1; \
	}
	git ls-files -z --cached --others --exclude-standard '*.sh' | \
	    xargs -0 --no-run-if-empty shellcheck -x -P SCRIPTDIR --
	shellcheck -x -P SCRIPTDIR -- .githooks/pre-commit .githooks/pre-push

# The scripts' own self-checks: a `*.test.sh` beside the script it exercises,
# run as a plain program, exit status is the verdict. They stub the commands
# that reach a device, so they touch no hardware and no network and belong in
# the gate beside the Rust tests.
#
# Their decisions are refusals — a stale binary, a held bus — and a refusal
# that stops working is a refusal nobody notices until a bench night is spent
# on the run it should have stopped.
#
# The set is the shellcheck sweep's, narrowed to the self-checks: every
# `*.test.sh` in the working tree, tracked or not, with the ignore rules
# honoured. A self-check that runs only once it is staged is a self-check whose
# first green run says nothing about the gate that will carry it.
#
# The trade that buys, stated because it is a trade and not an oversight: this
# lane *runs* what it finds, so the gate executes files nobody staged. A commit
# attempt is enough to run a `*.test.sh` that arrived in the tree by some other
# route — an archive unpacked in place, a tool writing outside the ignored
# paths. Every build output this repo produces lands in an ignored directory, so
# in practice the set is what the developer wrote; a tree with a file in it
# nobody wrote is a compromised tree either way.
#
# It arrives NUL-separated for the reason the shellcheck sweep's does, and
# through a loop rather than xargs because each script is a program to run
# rather than an argument to one.
#
# Two refusals by name before the run, because both conditions otherwise fail as
# something else. A tracked script deleted from the working tree but not yet
# `git rm`-ed is still in the index, so the walk names a path that is not
# there. And a just-written script routinely carries no mode bit, where
# `Permission denied` on a program nobody asked to run reads like a broken gate
# rather than like one `chmod` away from green.
.PHONY: test-scripts
test-scripts:
	@while IFS= read -r -d '' script; do \
	    echo "$$script"; \
	    [ -f "$$script" ] || { \
	        echo "$$script is tracked but missing from the working tree:" >&2; \
	        echo "    git rm $$script, or restore it" >&2; \
	        exit 1; \
	    }; \
	    [ -x "$$script" ] || { \
	        echo "$$script is not executable — the gate runs it as a program:" >&2; \
	        echo "    chmod +x $$script" >&2; \
	        exit 1; \
	    }; \
	    "./$$script"; \
	done < <(git ls-files -z --cached --others --exclude-standard '*.test.sh')

# The whole gate, one lane: the shell scripts' own checks, then every Bazel
# target in the tree — the eight Rust crates with their tests, the `.clk` schema
# compiles, the generated cog wrappers, and the deterministic-runner scenarios.
#
# The scripts go first: they are seconds of work, and a broken deploy script is
# not something to discover after a full test run. Sequential sub-makes rather
# than prerequisites, because prerequisites are unordered and `make -j check`
# would interleave the three — and the bazel refusal would land after the work it
# is there to save.
#
# --config=lint adds the clippy and rustfmt aspects over every Rust target, with
# warnings denied. A warning the gate tolerates is a warning nobody reads.
#
# BAZEL_FLAGS is how CI adds --lockfile_mode=error to the same invocation
# developers run: locally the lockfile refreshes as a side effect of a build,
# and the gate is the place that has to refuse the refreshed bytes rather than
# absorb them. One target, two flag sets, no second copy of the command.
#
# The cold-cache cost — the pinned Clockwork drop's whole dependency graph, a
# hermetic clang sysroot among it, minutes and gigabytes — is paid once per
# machine; on a warm cache the lane is seconds.
BAZEL_FLAGS ?=


# The refusal every Bazel-backed target here shares, in one place: a target that
# needs bazel asks for this first and gets the same explanation, rather than each
# one carrying its own copy.
#
# tools/build-bench.sh carries the one deliberate second copy: it is a script
# run outside make, and its REACHY_BAZEL knob gives it a third line to say.
.PHONY: require-bazel
require-bazel:
	@command -v bazel >/dev/null 2>&1 || { \
	    echo "bazel not found on PATH." >&2; \
	    echo "Install bazelisk; .bazelversion pins the Bazel release it fetches." >&2; \
	    exit 1; \
	}

.PHONY: check
check:
	@$(MAKE) require-bazel
	@$(MAKE) check-scripts
	@$(MAKE) test-scripts
	bazel test --config=lint $(BAZEL_FLAGS) //...

# The other half of the gate: everything a unit runs, built for the device. A
# build and not a run — nothing here needs a device, and nothing here executes
# an aarch64 binary.
#
# Separate from `check` because it is a separate configuration of the whole
# dependency graph, and a commit hook that paid for both would be a hook people
# route around. CI runs the two side by side and blocks on both, so a break
# lands on a pull request rather than at `make motion-build` in front of a
# powered-up unit: a dependency without aarch64 in its
# `target_compatible_with`, a `select()` a new dependency introduces, a drop
# upgrade that stops registering the aarch64 clang toolchain.
#
# The deployables are named once, in `//bazel/platform:device_deployables`, and
# `tools/build-motion.sh` builds a subset of that same list — so the gate cannot
# cover a different set than a deploy ships. One invocation, so one analysis
# phase serves them all: the cog executable's own C++ graph is most of the work,
# and the bench and the driver ride along on it.
#
# The build is not the whole of it: a binary that compiles for the device can
# still carry instructions the device's CPU cannot execute. So the build is
# followed by a disassembly of what it just produced, asserting the instruction
# set is the one that unit implements.
#
# `host_closure_test` is named beside the filegroup because it is testonly and a
# filegroup cannot carry that. It is the only target that links OpenSSL and ONNX
# Runtime, so it is where the voice pipeline's native libraries are proved to
# resolve for the unit's architecture — the link, and only the link: what the
# prebuilt ONNX Runtime's own instructions are is the ISA sweep's question, and
# it reads that object too.
#
# `identity_check` is named for the third question about that runtime: which
# release the aarch64 archive holds. Nothing else asks it — the closure test
# interrogates the linked library, and the only lane that links one is the
# workstation's x86_64 build — so without this the two archives could drift to
# different releases and the gate would stay green.
.PHONY: check-device
check-device: require-bazel
	bazel build --config=device $(BAZEL_FLAGS) -- \
	    //bazel/platform:device_deployables \
	    //bazel/third_party/onnxruntime:identity_check \
	    //crates/reachy-host:host_closure_test
	tools/assert-device-isa.sh

# Auto-fix, which means formatting: rules_rust's rustfmt runner formats every
# Rust target in the tree with the pinned toolchain's rustfmt, which is the same
# formatter the gate's rustfmt aspect checks with. Review the diff before
# committing.
#
# Clippy findings are fixed by hand from the gate's output. There is no Bazel
# equivalent of `clippy --fix`, and the gate never depended on one.
.PHONY: fix
fix: require-bazel
	bazel run @rules_rust//tools/rustfmt:target_aware_rustfmt

# Wire git at the tracked hooks directory and report any missing tooling.
# Idempotent; run once per clone.
#
# The scanners are external tools this tree cannot build, so this reports what is
# missing and where to get it rather than installing anything. A missing scanner
# is reported, never worked around: the commit and push gates simply do not run
# without it, which is the failure mode worth being loud about.
.PHONY: setup-hooks
setup-hooks:
	git config core.hooksPath .githooks
	@rm -f .git/hooks/pre-commit
	@command -v brenn-scrub >/dev/null 2>&1 || { \
	    echo "brenn-scrub not found on PATH — the commit and push gates will not run."; \
	    echo "Install it from the brenn repo: cargo install --path scrub"; \
	}
	@command -v gitleaks >/dev/null 2>&1 || { \
	    echo "gitleaks not found on PATH — brenn-scrub cannot scan without it."; \
	    echo "Install the release brenn-scrub pins; it refuses to run against any other."; \
	}
	@command -v bazel >/dev/null 2>&1 || { \
	    echo "bazel not found on PATH — the commit gate will not run."; \
	    echo "Install bazelisk; .bazelversion pins the Bazel release it fetches."; \
	}
	@echo "setup-hooks: done."

.PHONY: scrub-tree
scrub-tree:
	brenn-scrub tree

# Regenerate the clip library the cogs are handed, from the documents under
# `cogs/clips/`. Not part of the gate and not a prerequisite of anything: the
# emitted asset is committed, and the gate's own case fails when the two
# disagree, so this is the command that answers that failure.
#
# Absolute paths, because `bazel run` runs the tool in its runfiles tree rather
# than here.
.PHONY: clip-config
clip-config: require-bazel
	bazel run //cogs:gen_clip_config -- \
	    --clips $(CURDIR)/cogs/clips \
	    --out $(CURDIR)/cogs/clip_library.textproto \
	    --names $(CURDIR)/cogs/clip_library.names.json

# The online system, on this machine: the real control loop and the real logger,
# with the simulated plant behind the real UDP seam, all three under the launcher
# a unit uses. A fixed budget of wall clock — the script states it and derives it
# — then `first_motion_report` over the log the run wrote, and the target's
# verdict is the report's.
#
# What it is for is configuration, not physics: the composition, the process
# descriptions, the shared-memory namespace, the launcher config, the seam's wire
# contract and the log format are all otherwise first exercised at a powered
# unit. Not part of the gate — it is wall-clock-long and its flake envelope is
# unmeasured. The script's header says what a green run does and does not mean.
#
# Needs bazel; needs no device, and reaches none.
.PHONY: motion-host-run
motion-host-run: require-bazel
	tools/host-motion-run.sh

# ---------------------------------------------------------------------------
# The device path: building the bench for the Reachy Mini and running it there.
#
# None of this is part of the gate. It needs bazel to build and a reachable
# unit to run, and it exists because the device cannot build anything itself —
# brenn-os carries no compiler, and its flash is not somewhere a toolchain gets
# installed.

# The unit these targets talk to. Named for one invocation on the command line
# first, and in the gitignored .local/reachy.conf second, so a workstation that
# always talks to the same unit says so once:
#
#     echo 'REACHY_HOST=reachy00' > .local/reachy.conf
#
# The file is only read when the variable is not already set, so an override for
# one invocation stays an override for one invocation.
ifeq ($(origin REACHY_HOST), undefined)
-include .local/reachy.conf
endif

# The bench's configuration for this unit. Gitignored: it holds the serial node
# this machine's servos are on.
BENCH_CONFIG ?= .local/reachy-bench.toml

# Where fetched state files accumulate. One per run, named for its fetch time.
BENCH_RECORDS ?= .local/records

# Refuse before doing anything rather than ssh to nowhere. A prerequisite of
# every target that talks to a device, listed first so the refusal comes before
# a build.
.PHONY: device-host
device-host:
	@[ -n "$(REACHY_HOST)" ] || { \
	    echo "REACHY_HOST is not set, so there is no device to talk to." >&2; \
	    echo "Name the unit for this invocation:" >&2; \
	    echo "    make $(MAKECMDGOALS) REACHY_HOST=<hostname>" >&2; \
	    echo "or once, in the gitignored .local/reachy.conf:" >&2; \
	    echo "    REACHY_HOST=<hostname>" >&2; \
	    exit 1; \
	}

# The aarch64 binary, cross-compiled for the device platform. Needs bazel; needs
# no device.
.PHONY: bench-build
bench-build:
	tools/build-bench.sh

# Give the unit the bench's configuration. Needs no build, so it can run before
# or after one; idempotent, and the device's copy is in RAM.
#
# This is one file out of everything a reboot clears, and this target is true
# about itself only. Bringing a rebooted unit all the way back — payload, both
# daemons' configurations, the motion daemon's token, binary and unit — is
# `make reachy-up` in brenn-pod, which pushes this file too, out of the same
# BENCH_CONFIG copy. Its runbook, docs/runbooks/reachy-end-to-end.md there, is
# the prose version. Use this one when the bench file is the only thing that
# changed.
.PHONY: bench-config
bench-config: device-host
	tools/deploy-bench.sh $(REACHY_HOST) --config $(BENCH_CONFIG)

# Build, push, and run the bench on the unit. ARGS is the command and its
# arguments, passed verbatim:
#
#     make bench-run ARGS="reboot 17"
#
# The build is a prerequisite rather than a step to remember, which is what
# makes this the entry point that cannot go stale — deploy-bench.sh refuses a
# binary older than the newest commit, and this target is why that refusal
# should never fire.
.PHONY: bench-run
bench-run: device-host bench-config bench-build
	tools/deploy-bench.sh $(REACHY_HOST) --run $(ARGS)

# Build, push, and run the read-only self-test registry on the unit. ARGS is
# passed to the bench verbatim.
#
# Leave the record where it lands — bench-fetch reads it from beside the
# configuration. A --record elsewhere writes to RAM that no target brings
# back and a reboot clears.
.PHONY: bench-selftest
bench-selftest: device-host bench-config bench-build
	tools/deploy-bench.sh $(REACHY_HOST) --run selftest $(ARGS)

# Bring a run's state file back. Each fetch lands under its own timestamped
# name, so the several runs a session calls for accumulate rather than
# overwrite each other.
.PHONY: bench-fetch
bench-fetch: device-host
	tools/deploy-bench.sh $(REACHY_HOST) --fetch $(BENCH_RECORDS)

# ---------------------------------------------------------------------------
# The motion test: the cog system and its driver, on the unit.
#
# Not a variant of the bench path. A bench command is one invocation that ends; a
# motion run is three long-lived processes under a supervisor, on a machine that
# can move. `motion-run` is the one command that builds, pushes, runs, fetches
# and judges; the person in the loop is the operator who types it standing at the
# machine, and the bus refusals plus the runbook's eyes-on rule are what that
# presence is for. `docs/bench-runbook.md` is the procedure.

# Where fetched run logs accumulate. One directory per fetch, named for its
# fetch time.
MOTION_RECORDS ?= .local/motion-logs

# The aarch64 payload: the launcher, both binaries, the launcher config and every
# configuration file the three processes read, laid out the way the launcher
# config and those processes' relative paths expect. Needs bazel; needs no
# device.
.PHONY: motion-build
motion-build:
	tools/build-motion.sh

# Push the payload into the unit's RAM and make the directory the logger writes
# into. The build is a prerequisite rather than a step to remember, which is what
# makes this the entry point that cannot go stale — deploy-motion.sh refuses a
# payload older than the newest commit, and this target is why that refusal
# should never fire.
.PHONY: motion-deploy
motion-deploy: device-host motion-build
	tools/deploy-motion.sh $(REACHY_HOST) --push

# Build, push, run, fetch, judge. The run is given a fixed budget and stopped by
# it; the launcher's console streams here meanwhile, and this target's verdict is
# `first_motion_report`'s over the records it brought back. The runbook's manual
# appendix is the same run started by hand.
#
# Needs a reachable unit and bazel — the analyzer runs here, over the fetched log.
.PHONY: motion-run
motion-run: device-host motion-deploy require-bazel
	tools/deploy-motion.sh $(REACHY_HOST) --run $(MOTION_RECORDS)

# Bring a run's records back. Each fetch lands under its own timestamped
# directory, so a session's runs accumulate rather than overwrite.
.PHONY: motion-fetch
motion-fetch: device-host
	tools/deploy-motion.sh $(REACHY_HOST) --fetch $(MOTION_RECORDS)
