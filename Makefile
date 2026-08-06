# Repo-root Makefile — the check gate, the tree sweep, and hook wiring.
#
# `check` holds the contents of the gate, so both the pre-commit hook and CI
# invoke one definition rather than two copies that drift, and the whole gate is
# reproducible on a fresh clone. The secret sweep is a separate target, and
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
	@echo "  make check         the gate (fmt, clippy, tests) — same contents CI runs"
	@echo "  make fix           auto-fix what the gate can fix (fmt + clippy --fix)"
	@echo "  make setup-hooks   wire git at .githooks, check tooling (once per clone)"
	@echo "  make scrub-tree    whole-tree secret sweep — the sweep a clean tree is declared on"
	@echo ""
	@echo "Device targets — real hardware, no part of any gate. Need podman, and a"
	@echo "REACHY_HOST naming a reachable unit:"
	@echo "  make bench-build     the aarch64 binary, built in the pinned container"
	@echo "  make bench-config    push the bench's configuration into the unit's RAM"
	@echo "  make bench-selftest  build, push, run the read-only registry on the unit"
	@echo "  make bench-fetch     bring a run's state file back, timestamped"

# The whole gate. One workspace at the repo root, every crate a default member,
# so --workspace and a bare invocation cover the same set.
#
# -D warnings on clippy and --check on fmt: this repo's style rules are not
# advisory, and a warning that CI tolerates is a warning nobody reads.
.PHONY: check
check:
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cargo test --workspace

# Auto-fix, scoped to exactly what `check` enforces, so `make fix && make check`
# is always a clean cycle. --allow-dirty/--allow-staged so it is usable mid-edit;
# review the diff before committing.
.PHONY: fix
fix:
	cargo fmt --all
	cargo clippy --fix --allow-dirty --allow-staged --workspace --all-targets

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
	@echo "setup-hooks: done."

.PHONY: scrub-tree
scrub-tree:
	brenn-scrub tree

# ---------------------------------------------------------------------------
# The device path: building the bench for the Reachy Mini and running it there.
#
# None of this is part of the gate. It needs podman to build and a reachable
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
# and, once a person has reviewed a run, the crank datum record for one machine.
BENCH_CONFIG ?= .local/reachy-bench.toml

# Where fetched state files accumulate. One per run, named for its fetch time.
BENCH_RECORDS ?= .local/records

# Refuse before doing anything rather than ssh to nowhere. A prerequisite of
# every target that talks to a device, listed first so the refusal comes before
# a container build.
.PHONY: bench-host
bench-host:
	@[ -n "$(REACHY_HOST)" ] || { \
	    echo "REACHY_HOST is not set, so there is no device to talk to." >&2; \
	    echo "Name the unit for this invocation:" >&2; \
	    echo "    make $(MAKECMDGOALS) REACHY_HOST=<hostname>" >&2; \
	    echo "or once, in the gitignored .local/reachy.conf:" >&2; \
	    echo "    REACHY_HOST=<hostname>" >&2; \
	    exit 1; \
	}

# The aarch64 binary, built in the pinned container. Needs podman; needs no
# device.
.PHONY: bench-build
bench-build:
	tools/build-bench.sh

# Give the unit the bench's configuration. Needs no build, so it can run before
# or after one; idempotent, and the re-run after a reboot is the whole story —
# the device's copy is in RAM.
.PHONY: bench-config
bench-config: bench-host
	tools/deploy-bench.sh $(REACHY_HOST) --config $(BENCH_CONFIG)

# Build, push, and run the read-only self-test registry on the unit. ARGS is
# passed to the bench verbatim.
#
# Leave the record where it lands — bench-fetch reads it from beside the
# configuration. A --record elsewhere writes to RAM that no target brings
# back and a reboot clears.
.PHONY: bench-selftest
bench-selftest: bench-host bench-build
	tools/deploy-bench.sh $(REACHY_HOST) --run selftest $(ARGS)

# Bring a run's state file back. Each fetch lands under its own timestamped
# name, so the several runs a session calls for accumulate rather than
# overwrite each other.
.PHONY: bench-fetch
bench-fetch: bench-host
	tools/deploy-bench.sh $(REACHY_HOST) --fetch $(BENCH_RECORDS)
