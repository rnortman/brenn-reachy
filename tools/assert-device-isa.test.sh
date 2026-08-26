#!/usr/bin/env bash
#
# tools/assert-device-isa.test.sh — self-check for assert-device-isa.sh.
#
# The subject is the only automated guard on the device binaries' instruction
# set, and its whole value is the red verdict: a guard that has quietly lost the
# ability to fail is worse than none, because it retires the suspicion that
# `make check-device` can be green on binaries that die on the unit. So most of
# the cases here are the red ones.
#
# Nothing here runs bazel, a disassembler, or an aarch64 binary. Both are
# stubbed: `bazel cquery` answers with paths into a temporary tree, and
# `REACHY_OBJDUMP` names a stub that prints a disassembly this file wrote. The
# fixtures are the two shapes that matter — an LSE instruction behind the
# outline-atomics dispatch, and one in straight-line code — written out as
# llvm-objdump prints them.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"


# ---------------------------------------------------------------------------
# The tree the subject runs out of
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/bazel-out/bin"
cp -- "${script_dir}/assert-device-isa.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/assert-device-isa.sh"

# The files cquery names. Their contents are never read — the disassembler is a
# stub — but they have to be there, because the subject refuses a path the build
# claims and the tree does not hold.
: >"${repo}/bazel-out/bin/simplelaunch"
: >"${repo}/bazel-out/bin/robot_clk_exe"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# The bazel stub: one subcommand the subject uses, `cquery`, answering with the
# listing shape the real one produces — the outputs of the filegroup plus the
# sources that came along with it, so the subject's basename matching has
# something to get wrong.
export CQUERY_STATUS=""
export CQUERY_DROP=""
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
case $1 in
cquery)
	[ -z "${CQUERY_STATUS:-}" ] || exit "$CQUERY_STATUS"
	echo bazel-out/bin/reachy_motord
	[ "${CQUERY_DROP:-}" = simplelaunch ] || echo bazel-out/bin/simplelaunch
	[ "${CQUERY_DROP:-}" = robot_clk_exe ] || echo bazel-out/bin/robot_clk_exe
	echo cogs/robot.clk
	;;
info) echo "${FIXTURES}/output_base" ;;
*) echo "unstubbed subcommand $1" >&2; exit 1 ;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/bazel"

# The disassembler stub: prints the fixture named for the binary it was asked
# about, so a case chooses what the subject sees by writing that file. Exits
# non-zero when a case says to, which is the one failure mode a real objdump has
# that the subject has to report rather than read past.
export OBJDUMP_STATUS=""
export FIXTURES="${work}/fixtures"
mkdir -p -- "$FIXTURES"
cat >"${stubs}/stub-objdump" <<'STUB'
#!/usr/bin/env bash
[ -z "${OBJDUMP_STATUS:-}" ] || exit "$OBJDUMP_STATUS"
for arg in "$@"; do
	case "$arg" in
	-*) continue ;;
	esac
	cat -- "${FIXTURES}/$(basename -- "$arg")"
done
STUB
chmod 0755 -- "${stubs}/stub-objdump"
export REACHY_OBJDUMP=stub-objdump

# ---------------------------------------------------------------------------
# The fixtures
# ---------------------------------------------------------------------------

# The outline-atomics dispatch, as compiler-rt emits it and llvm-objdump prints
# it: the HWCAP byte, the branch that skips the LSE instruction when the CPU
# lacks the feature, the instruction itself, and the LL/SC loop it branched
# over. Allowed, and the reason the check cannot be a plain mnemonic grep.
guarded() {
	cat <<'DUMP'
0000000000391810 <__aarch64_ldadd8_acq_rel>:
  391810:      	adrp	x16, 0x3ba000
  391814:      	ldrb	w16, [x16, #0x7ac]
  391818:      	cbz	w16, 0x391824 <__aarch64_ldadd8_acq_rel+0x14>
  39181c:      	ldaddal	x0, x0, [x1]
  391820:      	ret
  391824:      	mov	x16, x0
  391828:      	ldaxr	x0, [x1]
  39182c:      	add	x17, x0, x16
  391830:      	stlxr	w15, x17, [x1]
  391834:      	cbnz	w15, 0x391828 <__aarch64_ldadd8_acq_rel+0x18>
  391838:      	ret
DUMP
}

# A disassembly's opening line, which is where the subject reads the
# architecture it is looking at.
header() {
	printf '%s:\tfile format %s\n\nDisassembly of section .text:\n\n' "$1" "${2:-elf64-littleaarch64}"
}

# One binary's fixture: the header, the guarded helper, and whatever extra
# lines a case wants after it.
fixture() {
	local name=$1
	shift
	{
		header "$name"
		guarded
		[ "$#" -eq 0 ] || printf '%s\n' "$@"
	} >"${FIXTURES}/${name}"
}

# The green pair: nothing but the guarded helpers in either binary.
green() {
	fixture simplelaunch
	fixture robot_clk_exe
}

# ---------------------------------------------------------------------------
# The runs
# ---------------------------------------------------------------------------

# The subject's stdout, stderr and status as one string, so a case can assert on
# all three. Run from a directory that is not the repo, because the subject cds
# to its own root and a case that passed only from inside it would prove nothing.
run() {
	local out status
	set +e
	out=$(cd -- "$work" && "$subject" 2>&1)
	status=$?
	set -e
	printf '%s\n---status %s\n' "$out" "$status"
}

# ---------------------------------------------------------------------------
# The green verdict
# ---------------------------------------------------------------------------

green
result=$(run)
assert_status "both binaries clean is a pass" 0 "$(status_of "$result")"
assert_contains "and it says so per binary" "$(output_of "$result")" \
	"simplelaunch: no unguarded ARMv8.1 atomics."
assert_contains "including the second one" "$(output_of "$result")" \
	"robot_clk_exe: no unguarded ARMv8.1 atomics."

# The LL/SC instructions the guarded fixture also carries — `ldaxr`, `stlxr`,
# `cbnz` — are what a correctly compiled binary is full of, and none of them is
# an LSE atomic. A check that matched on a prefix or a substring would have
# failed the green case above; this says which lines were on trial.
assert_lacks "an LL/SC loop is not a finding" "$(output_of "$result")" "ldaxr"

# ---------------------------------------------------------------------------
# The red verdicts
# ---------------------------------------------------------------------------

# The defect this exists for: a straight-line LSE atomic, which is what the drop
# 's toolchain emits at its own -march and what died with SIGILL on the unit.
fixture simplelaunch \
	'0000000000401000 <_Z3foov>:' \
	'  401000:      	ldr	x8, [x0]' \
	'  401004:      	ldaddal	x1, x2, [x8]' \
	'  401008:      	ret'
result=$(run)
assert_status "an unguarded LSE atomic fails" 1 "$(status_of "$result")"
assert_contains "naming the binary" "$(output_of "$result")" \
	"simplelaunch carries instructions this unit's CPU does not implement"
assert_contains "and the instruction, with its address" "$(output_of "$result")" \
	"401004  ldaddal	x1, x2, [x8]"
assert_contains "and where the pin lives" "$(output_of "$result")" \
	".bazelrc's build:device -march/-mcpu pair"
assert_contains "the clean binary still reports clean" "$(output_of "$result")" \
	"robot_clk_exe: no unguarded ARMv8.1 atomics."

# Every family the check covers, in one binary: compare-and-swap, swap, and the
# read-modify-write forms with and without their width and ordering suffixes.
# Counted, because the count is what tells an operator whether this is one stray
# instruction or a whole configuration compiled wrong.
green
fixture robot_clk_exe \
	'0000000000401000 <_Z3barv>:' \
	'  401000:      	cas	w0, w1, [x2]' \
	'  401004:      	casal	x0, x1, [x2]' \
	'  401008:      	casb	w0, w1, [x2]' \
	'  40100c:      	swpal	x0, x1, [x2]' \
	'  401010:      	ldsetl	x0, x1, [x2]' \
	'  401014:      	stadd	x0, [x2]' \
	'  401018:      	ldumaxa	w0, w1, [x2]' \
	'  40101c:      	ret'
result=$(run)
assert_status "every LSE family is a finding" 1 "$(status_of "$result")"
assert_contains "counted" "$(output_of "$result")" "7 unguarded ARMv8.1 LSE atomics"

# A `cbz` alone is not the dispatch. Without the HWCAP byte the branch tested,
# the instruction after it is reachable on any CPU — so the shape is matched,
# not just the branch.
green
fixture simplelaunch \
	'0000000000401000 <_Z3bazv>:' \
	'  401000:      	cbz	w16, 0x401008 <_Z3bazv+0x8>' \
	'  401004:      	casal	x0, x1, [x2]' \
	'  401008:      	ret'
result=$(run)
assert_status "a bare cbz does not excuse an LSE atomic" 1 "$(status_of "$result")"
assert_contains "the instruction is still named" "$(output_of "$result")" \
	"401004  casal"

# A symbol header carrying an LSE name is not an instruction. The real
# disassembly is full of `__aarch64_cas8_acq` labels, and a check reading them
# as code would be red on every correctly built binary.
green
fixture simplelaunch \
	'0000000000391510 <__aarch64_cas4_relax>:' \
	'  391510:      	ret'
result=$(run)
assert_status "a symbol name is not an instruction" 0 "$(status_of "$result")"

# ---------------------------------------------------------------------------
# The refusals: what the subject does when it cannot observe
# ---------------------------------------------------------------------------

# A host binary at the path the device build was supposed to write. The device
# configuration's output directory is named for a cpu flag the platform never
# sets, so it reads like a host build's, and disassembling the wrong machine
# would pass this check for the worst possible reason.
green
{
	header simplelaunch elf64-x86-64
	printf '  401000:      	retq\n'
} >"${FIXTURES}/simplelaunch"
result=$(run)
assert_status "a non-aarch64 file is refused" 1 "$(status_of "$result")"
assert_contains "saying which build was read" "$(output_of "$result")" \
	"is not an aarch64 binary, so this checked the wrong build"

# An empty disassembly is not a clean one.
green
: >"${FIXTURES}/simplelaunch"
result=$(run)
assert_status "an empty disassembly is refused" 1 "$(status_of "$result")"
assert_contains "saying nothing was checked" "$(output_of "$result")" \
	"so this checked nothing"

# The disassembler failing on its own account.
green
OBJDUMP_STATUS=1
result=$(run)
OBJDUMP_STATUS=""
assert_status "a failed disassembly is refused" 1 "$(status_of "$result")"
assert_contains "naming the tool" "$(output_of "$result")" "cannot disassemble"

# A named disassembler that is not there. The knob exists for a machine whose
# drop has moved; a typo in it must not read as a clean tree.
green
REACHY_OBJDUMP="${work}/no-such-objdump"
result=$(run)
REACHY_OBJDUMP=stub-objdump
assert_status "an unusable REACHY_OBJDUMP is refused" 1 "$(status_of "$result")"
assert_contains "naming the value" "$(output_of "$result")" "no-such-objdump"

# ---------------------------------------------------------------------------
# Resolving the disassembler with no REACHY_OBJDUMP set
# ---------------------------------------------------------------------------
#
# The knob is how this suite chooses what the subject sees, so every case above
# sets it -- and the branch `make check-device` actually takes on CI and on a
# workstation is the one nothing above exercises. A candidate on PATH, the
# pinned drop's own copy, and neither, each asserted.
#
# The PATH for these is assembled rather than prepended to, because a
# workstation with a real llvm-objdump installed would otherwise answer the
# cases that are about there being none: it carries the stub bazel, this stub
# disassembler under the names the subject looks for, and symlinks to the
# handful of programs the subject and lib.sh run.
sanitized="${work}/sanitized-bin"
mkdir -p -- "$sanitized"
for tool in bash env cat basename dirname awk sed tail head wc tr; do
	ln -sf -- "$(command -v -- "$tool")" "${sanitized}/${tool}"
done
ln -sf -- "${stubs}/bazel" "${sanitized}/bazel"

# The drop's own copy, where `bazel info output_base` says the clang is.
drop="${FIXTURES}/output_base/external/clang+/usr/bin"
mkdir -p -- "$drop"
cp -- "${stubs}/stub-objdump" "${drop}/llvm-objdump"

# An llvm-objdump on PATH is what runs. The drop's copy is moved out of the way
# for this one: both stubs disassemble identically, so with the fallback in
# place a broken candidate list would still pass here on the drop's answer.
green
saved_path=$PATH
on_path="${work}/on-path-bin"
mkdir -p -- "$on_path"
cp -- "${stubs}/stub-objdump" "${on_path}/llvm-objdump"
mv -- "${drop}/llvm-objdump" "${FIXTURES}/objdump.moved"
PATH="${on_path}:${sanitized}"
unset REACHY_OBJDUMP
result=$(run)
PATH=$saved_path
export REACHY_OBJDUMP=stub-objdump
mv -- "${FIXTURES}/objdump.moved" "${drop}/llvm-objdump"
assert_status "an llvm-objdump on PATH is found with no knob set" 0 "$(status_of "$result")"
assert_contains "and it disassembled both binaries" "$(output_of "$result")" \
	"robot_clk_exe: no unguarded ARMv8.1 atomics."

# Nothing on PATH: the drop's own copy is the fallback, which is the point of
# preferring an llvm-objdump at all -- a distribution objdump is routinely built
# for the host architecture only.
green
PATH=$sanitized
unset REACHY_OBJDUMP
result=$(run)
PATH=$saved_path
export REACHY_OBJDUMP=stub-objdump
assert_status "the pinned drop's own disassembler is the fallback" 0 "$(status_of "$result")"
assert_contains "and it disassembled both binaries" "$(output_of "$result")" \
	"simplelaunch: no unguarded ARMv8.1 atomics."

# No disassembler anywhere is a refusal that names the way out. A guard that
# cannot disassemble must not read as a clean tree, and this is the branch a
# machine whose drop layout moved lands on.
green
mv -- "${drop}/llvm-objdump" "${FIXTURES}/objdump.moved"
PATH=$sanitized
unset REACHY_OBJDUMP
result=$(run)
PATH=$saved_path
export REACHY_OBJDUMP=stub-objdump
mv -- "${FIXTURES}/objdump.moved" "${drop}/llvm-objdump"
assert_status "no disassembler anywhere is refused" 1 "$(status_of "$result")"
assert_contains "and the refusal names the knob that answers it" "$(output_of "$result")" \
	"Set REACHY_OBJDUMP"

# ---------------------------------------------------------------------------
# The refusals, continued
# ---------------------------------------------------------------------------

# A build that emits no launcher. Something upstream renamed it, and the check
# has nothing to look at — which is a refusal, not a pass.
green
CQUERY_DROP=simplelaunch
result=$(run)
CQUERY_DROP=""
assert_status "a missing deployable is refused" 1 "$(status_of "$result")"
assert_contains "naming what is missing" "$(output_of "$result")" \
	"the build emits no simplelaunch"

# bazel itself failing.
green
CQUERY_STATUS=1
result=$(run)
CQUERY_STATUS=""
assert_status "a failed cquery is refused" 1 "$(status_of "$result")"
assert_contains "naming the query" "$(output_of "$result")" \
	"bazel cannot name the outputs of //bazel/platform:device_deployables"

tally
