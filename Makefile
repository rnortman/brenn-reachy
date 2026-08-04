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
	@echo "The bench binary needs real hardware and is not part of any gate."
	@echo "It parses no arguments yet, and every invocation exits nonzero saying so."

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
