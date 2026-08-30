#!/usr/bin/env bash
#
# Assert that the device binaries can start and run on the unit: no instruction
# the CPU cannot execute, and nothing the loader cannot resolve.
#
#   tools/assert-device-isa.sh
#
# Two checks over one build, sharing one disassembler. The first is the ISA
# sweep below. The second is the voice host's loader contract, at the bottom:
# both are the class of defect that builds green, stages green, pushes green,
# and shows up as a process that dies on a powered unit.
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
# the gate builds or a deploy ships, plus the prebuilt ONNX Runtime the voice
# host loads. The loader check reads the voice host out of the same filegroup.
# It builds nothing: run it after that build.
#
# Knobs, environment only:
#
#   REACHY_BAZEL     the bazel to run (default bazel)
#   REACHY_OBJDUMP   the disassembler (default: an llvm-objdump on PATH, else
#                    the pinned drop's own)
#
# Exits 0 with a per-binary count and the host's loader contract, or non-zero
# naming the binary and what is wrong with it.

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

# The prebuilt ONNX Runtime, checked for the opposite reason: it is a
# third-party aarch64 object whose -march nobody in this tree chose, fetched by
# hash and loaded by the voice host at its first inference. Nothing else the
# unit runs is outside this repo's own compile flags, so it is the one payload
# member where the SIGILL this check exists to prevent could arrive without any
# .bazelrc change to explain it. Its path is inside the fetched repository, which
# is reached through the execution root rather than the bazel-out symlink the
# deployables resolve through.
shared_object_target=//bazel/third_party/onnxruntime:shared_object
shared_object_name=libonnxruntime.so.1

# The voice host, and what its exec depends on. The speech pipeline links ONNX
# Runtime dynamically, so the host carries a `NEEDED` for the shared object and
# a runpath ending in `$ORIGIN`; the payload stages the two side by side and the
# launcher runs the host from that directory. Three files state that in prose --
# the crate's BUILD file, the build script's staging, the deploy script's
# preflight -- and until this check nothing asked the binary itself. A dropped
# `rustc_flags` line, a rules_rust change in how link args reach the linker, or
# a linker default that stops writing the tag produces a payload that passes
# every other gate and a host that dies at exec with a loader message and no
# narration at all.
loader_binary=reachy_host
loader_needed=libonnxruntime.so.1
# shellcheck disable=SC2016 # `$ORIGIN` is the loader's own syntax, not this shell's
loader_runpath='$ORIGIN'

# The host's dynamic section, checked against that contract. Reports the first
# thing wrong and nothing after it: either half missing is the same fix.
check_loader_contract() {
	local path=$1 dynamic
	dynamic=$("$objdump" -p "$path" 2>/dev/null) ||
		die "${objdump} cannot read ${path}, so the loader contract was not checked."
	grep -q 'Dynamic Section' <<<"$dynamic" ||
		die "${loader_binary} has no dynamic section, so it links nothing dynamically." \
			"The voice host is expected to load ${loader_needed} at run time."
	grep -qE "^[[:space:]]*NEEDED[[:space:]]+${loader_needed}\$" <<<"$dynamic" || die \
		"${loader_binary} carries no NEEDED entry for ${loader_needed}." \
		"The payload stages that shared object beside the binary for this and nothing else;" \
		"a host that does not name it is one whose ONNX Runtime came from somewhere else."
	# The tag Bazel writes names a path inside the build tree, which is nowhere
	# on a unit; the crate appends `$ORIGIN` to it. So what has to hold is that
	# `$ORIGIN` is one of the colon-separated entries, not that it is the whole
	# value.
	local paths
	paths=$(awk '$1 == "RUNPATH" || $1 == "RPATH" { print $2 }' <<<"$dynamic")
	tr ':' '\n' <<<"$paths" | grep -Fxq -- "$loader_runpath" || die \
		"${loader_binary} has no ${loader_runpath} in its runpath (it reads '${paths:-nothing}')." \
		"The payload puts ${loader_needed} beside the binary and nowhere the loader searches by" \
		"default, so without it the host dies at exec." \
		"Check the rustc_flags in crates/reachy-host/BUILD.bazel."
	echo "${loader_binary}: NEEDED ${loader_needed}, runpath carries ${loader_runpath}."
}

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

# Everything to disassemble, as `name<tab>path` rows: the deployables out of the
# filegroup, then the shared object out of its own target.
subjects=()
for name in "${binaries[@]}"; do
	subjects+=("${name}"$'\t'"$(bazel_named_in "$listing" "$name")")
done

execroot=$("$bazel" info "${build_flags[@]}" execution_root \
	--ui_event_filters=-info --noshow_progress) ||
	die "bazel cannot say where its execution root is, so the fetched ${shared_object_name} cannot be found."
shared_object="${execroot}/$(bazel_files "$shared_object_target")"
[ -f "$shared_object" ] ||
	die "the build names ${shared_object} and no file is there." \
		"Run 'make check-device' first: this reads that build's inputs."
subjects+=("${shared_object_name}"$'\t'"${shared_object}")

status=0
for subject in "${subjects[@]}"; do
	IFS=$'\t' read -r name path <<<"$subject"
	bad=$(unguarded_lse "$path")
	if [ -n "$bad" ]; then
		status=1
		echo "${prog}: ${name} carries instructions this unit's CPU does not implement." >&2
		echo "    $(wc -l <<<"$bad" | tr -d ' ') unguarded ARMv8.1 LSE atomics, the first few:" >&2
		head -5 <<<"$bad" | while IFS=$'\t' read -r at insn; do
			printf '        %s  %s\n' "$at" "$insn" >&2
		done
		if [ "$name" = "$shared_object_name" ]; then
			echo "    This one is not compiled here: the pinned prebuilt is built above the" >&2
			echo "    Cortex-A72. Pin an ONNX Runtime release whose aarch64 build is not, or" >&2
			echo "    build one (MODULE.bazel names the archives and their hashes)." >&2
		else
			echo "    The device configuration compiles above the Cortex-A72 again: check that" >&2
			echo "    .bazelrc's build:device -march/-mcpu pair is still last on the compile" >&2
			echo "    line (bazel aquery --config=device 'mnemonic(\"CppCompile\", ...)')." >&2
		fi
		echo "    A binary in this state dies with SIGILL on the unit." >&2
	else
		echo "${name}: no unguarded ARMv8.1 atomics."
	fi
done

# Last, after the sweep has said what it found: the two checks are independent,
# and a run that reports both is worth more than one that stops at the first.
check_loader_contract "$(bazel_named_in "$listing" "$loader_binary")"

exit "$status"
