#!/usr/bin/env bash
#
# Assert that the device C++ deployables carry no instruction the unit's CPU
# cannot execute.
#
#   tools/assert-device-isa.sh
#
# The unit is a Cortex-A72: ARMv8.0-A plus CRC32. The pinned Clockwork drop's
# toolchain compiles at `-march=armv8.2-a+fp16+simd+dotprod+ssbs` unless
# something says otherwise, and .bazelrc's `device` config is what says
# otherwise. Building a binary says nothing about whether it can run — a
# compilable binary full of ARMv8.1 atomics dies with SIGILL on this CPU.
#
# So this is the durable half of that pin: a copt can be out-ordered by a
# toolchain flag silently, and a drop upgrade can move the default again, but a
# disassembly that still holds `cas` cannot be argued with. LSE is the whole
# check because it is what this toolchain reaches for first and by a wide margin
# — every atomic in boost, protobuf and abseil — so a binary compiled above the
# A72 has thousands of them, and one compiled at the A72 has only the guarded
# ones below.
#
# **Guarded LSE is allowed, and has to be.** Rust's compiler-rt links in the
# outline-atomics helpers, which is runtime dispatch: a HWCAP byte is loaded, a
# branch skips the LSE instruction when the CPU lacks it, and an LL/SC loop
# follows. Those instructions are in the binary and are never executed on an
# A72 — `reachy_motord` carries twenty of them and runs on the unit.
# `robot_clk_exe` carries the same helpers, linked in from the cog crate. The
# shape is fixed and recognisable, so it is recognised rather than excused:
#
#     adrp    x16, <hwcap page>
#     ldrb    w16, [x16, #<off>]
#     cbz     w16, <the LL/SC path>
#     ldaddal x0, x0, [x1]        <- allowed: unreachable without the feature
#
# An LSE instruction not in that shape is a straight-line use of a feature this
# CPU does not implement, which is a SIGILL waiting for a hardware session.
#
# Reads the two binaries out of the same `//bazel/platform:device_deployables`
# filegroup `make check-device` builds, so it cannot check a different set than
# the gate builds or a deploy ships. It builds nothing: run it after that build.
#
# Knobs, environment only:
#
#   REACHY_BAZEL     the bazel to run (default bazel)
#   REACHY_OBJDUMP   the disassembler (default: an llvm-objdump on PATH, else
#                    the pinned drop's own)
#
# Exits 0 with a per-binary count, or non-zero naming the binary and the first
# unguarded instructions found.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

bazel=${REACHY_BAZEL:-bazel}

# Same configuration the gate builds, because the output paths cquery answers
# with are a configuration's paths: asked in any other one, they name files from
# a build nobody ran. BAZEL_FLAGS is deliberately not threaded through — this is
# a query over a build that already happened, not a second build for a gate to
# constrain.
build_flags=(--config=device)

cd -- "$repo_root"

# The two C++ deployables. The Rust binaries are not checked here: their target
# is baseline ARMv8.0-A already, and what LSE they carry is the guarded helpers
# above, which is what this check has to tolerate rather than what it looks for.
deployables_target=//bazel/platform:device_deployables
binaries=(simplelaunch robot_clk_exe)

# The LSE atomic families, by mnemonic: compare-and-swap, swap, and the
# read-modify-write pair (the `st` forms are the same instructions with the
# result discarded). Optional acquire/release suffix, optional byte or halfword
# width. Anchored both ends, against a mnemonic field alone, so nothing matches
# on an operand or a symbol name that happens to read like one.
lse_mnemonics='^(cas|casp|swp|(ld|st)(add|clr|eor|set|smax|smin|umax|umin))(a|l|al)?(b|h)?$'

# The disassembler. An llvm-objdump knows every target LLVM does, which is what
# makes the drop's own copy the reliable fallback: a distribution binutils
# objdump is routinely built for the host architecture only, and would refuse an
# aarch64 file with a message about the file rather than about itself.
resolve_objdump() {
	local candidate found
	if [ -n "${REACHY_OBJDUMP:-}" ]; then
		command -v -- "$REACHY_OBJDUMP" >/dev/null 2>&1 ||
			die "REACHY_OBJDUMP names ${REACHY_OBJDUMP}, which is not executable."
		echo "$REACHY_OBJDUMP"
		return
	fi
	for candidate in llvm-objdump llvm-objdump-21 llvm-objdump-20 llvm-objdump-19; do
		command -v -- "$candidate" >/dev/null 2>&1 || continue
		echo "$candidate"
		return
	done
	local base
	base=$("$bazel" info output_base 2>/dev/null) ||
		die "no llvm-objdump on PATH and bazel cannot say where the drop's clang is." \
			"Set REACHY_OBJDUMP to a disassembler that knows aarch64."
	for found in "${base}/external/clang+/usr/bin/"llvm-objdump*; do
		[ -x "$found" ] || continue
		echo "$found"
		return
	done
	die "no llvm-objdump on PATH and none in the drop's clang at ${base}/external/clang+." \
		"Set REACHY_OBJDUMP to a disassembler that knows aarch64."
}

# Every LSE instruction in one binary that is not behind the outline-atomics
# dispatch, as `address<tab>mnemonic` lines.
#
# The dispatch is recognised by its last two instructions rather than all three:
# the `cbz` on the byte the `ldrb` just read is the branch that makes the LSE
# instruction unreachable without the feature, and the `adrp` ahead of them only
# says which page the byte is on. Registers are matched loosely — compiler-rt
# uses w16 today and the check should not turn red over a register allocation.
#
# One streamed pass, and nothing here ever holds the disassembly: an opt payload
# binary disassembles to on the order of a hundred megabytes of text, and this
# runs in a gate. Both questions — is this an aarch64 file at all, and did it
# disassemble to anything — are answered by the same awk program, on a summary
# line it prints last. What is buffered is the findings, which on a correctly
# built binary is nothing.
unguarded_lse() {
	local binary=$1 scanned summary
	scanned=$("$objdump" -d --no-show-raw-insn "$binary" 2>/dev/null | awk -v lse="$lse_mnemonics" '
		# The opening line names the file format, which is where the
		# architecture is read: the device configuration writes into an
		# output directory named for a cpu flag the platform never sets,
		# so it reads `k8-opt` like a host build would, and a path that
		# quietly named a host binary must not pass by disassembling the
		# wrong machine.
		/file format elf64-.*aarch64/ { aarch64 = 1 }
		# Instruction lines only: "  4008ec: <mnemonic> <operands>". Symbol
		# headers, section banners and the "..." elisions carry no address colon.
		$1 !~ /^[0-9a-f]+:$/ { prev2 = ""; prev1 = ""; next }
		{
			instructions++
			rest = $0
			sub(/^[^:]*:[[:space:]]*/, "", rest)
			if ($2 ~ lse &&
			    !(prev1 ~ /^cbz[[:space:]]+w[0-9]+,/ &&
			      prev2 ~ /^ldrb[[:space:]]+w[0-9]+,[[:space:]]*\[x[0-9]+/)) {
				printf "%s\t%s\n", substr($1, 1, length($1) - 1), rest
			}
			prev2 = prev1
			prev1 = rest
		}
		END { printf "scanned %d %s\n", instructions, aarch64 ? "aarch64" : "foreign" }
	') || die "${objdump} cannot disassemble ${binary}."
	# The summary line is the last one, and the findings are everything above
	# it -- the same shape the self-checks read a subject's status with.
	summary=$(tail -n 1 <<<"$scanned")
	[ "$summary" != "scanned 0 foreign" ] ||
		die "${objdump} disassembled ${binary} to nothing, so this checked nothing."
	[ "${summary##* }" = aarch64 ] ||
		die "${binary} is not an aarch64 binary, so this checked the wrong build." \
			"Run 'make check-device' first: this reads that build's outputs."
	sed '$d' <<<"$scanned"
}

objdump=$(resolve_objdump)

listing=$(bazel_files "$deployables_target")

status=0
for name in "${binaries[@]}"; do
	path=$(bazel_named_in "$listing" "$name")
	bad=$(unguarded_lse "$path")
	if [ -n "$bad" ]; then
		status=1
		echo "${prog}: ${name} carries instructions this unit's CPU does not implement." >&2
		echo "    $(wc -l <<<"$bad" | tr -d ' ') unguarded ARMv8.1 LSE atomics, the first few:" >&2
		head -5 <<<"$bad" | while IFS=$'\t' read -r at insn; do
			printf '        %s  %s\n' "$at" "$insn" >&2
		done
		echo "    The device configuration compiles above the Cortex-A72 again: check that" >&2
		echo "    .bazelrc's build:device -march/-mcpu pair is still last on the compile" >&2
		echo "    line (bazel aquery --config=device 'mnemonic(\"CppCompile\", ...)')." >&2
		echo "    A binary in this state dies with SIGILL on the unit." >&2
	else
		echo "${name}: no unguarded ARMv8.1 atomics."
	fi
done

exit "$status"
