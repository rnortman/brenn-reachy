"""One deterministic-runner scenario's four targets, as one macro.

A scenario of a cog system is always the same four things: a library stating
what the run is, an author that turns that statement into an input log, a
checker that joins the output log back to it, and a test that runs the three
phases against one system. Only the statement, the author and the checker are
about the scenario; the wiring between them -- which binary the harness script
gets in which position, and which of the system's build outputs it needs on
disk -- is the harness's protocol, and this is where it is stated.

Stated once, because it is the harness's and not the scenario's: adding a phase,
renaming a generated artifact or teaching the runner a new flag is then one edit
rather than one per scenario, and a scenario that fell out of step with the
others would fail its build rather than its run.
"""

load("@rules_rust//rust:defs.bzl", "rust_binary", "rust_library")
load("@rules_shell//shell:sh_test.bzl", "sh_test")

def scenario_suite(
        name,
        exe,
        process_description,
        channel_publisher_config,
        log_writer_config,
        configs = [],
        scenario_deps = [],
        author_deps = [],
        checker_deps = [],
        size = "small"):
    """Declare one scenario over one system.

    The three source files are `<name>_scenario.rs`, `<name>_author.rs` and
    `<name>_checker.rs`, in the package that calls this.

    Args:
      name: the scenario's prefix, which is also the pinion namespace its run
        gets -- a namespace of its own is what keeps two runs from sharing
        shared memory.
      exe: the system's process executable.
      process_description: the generated description that executable is run
        against.
      channel_publisher_config: the generated config saying which channels the
        input log publishes.
      log_writer_config: the generated config saying what the output log holds.
      configs: the config files the process reads, passed on to the checker as
        well so it can assert the run was configured the way the scenario is
        written for.
      scenario_deps: what the statement of the run needs beyond `:scenario`.
      author_deps: what the author needs beyond the statement, `:scenario` and
        the log writer.
      checker_deps: what the checker needs beyond the statement and `:scenario`.
      size: the test's size, for a scenario long enough to need more than the
        default.
    """
    scenario = name + "_scenario"
    author = name + "_author"
    checker = name + "_checker"

    rust_library(
        name = scenario,
        srcs = [scenario + ".rs"],
        edition = "2024",
        deps = [":scenario"] + scenario_deps,
    )

    rust_binary(
        name = author,
        srcs = [author + ".rs"],
        edition = "2024",
        deps = [
            ":" + scenario,
            ":scenario",
            "@rusty_cogs//crates/clockwork-logs:clockwork_logs",
        ] + author_deps,
    )

    rust_binary(
        name = checker,
        srcs = [checker + ".rs"],
        edition = "2024",
        deps = [
            ":" + scenario,
            ":scenario",
        ] + checker_deps,
    )

    # The order is the script's own argument list, and every path in it is
    # runfiles-relative -- which is what `data` below makes true.
    ordered = [
        ":" + author,
        exe,
        process_description,
        channel_publisher_config,
        log_writer_config,
        ":" + checker,
    ] + configs

    sh_test(
        name = name + "_test",
        size = size,
        srcs = ["scenario_test.sh"],
        args = [scenario] + ["$(rootpath %s)" % label for label in ordered],
        data = ordered,
    )
