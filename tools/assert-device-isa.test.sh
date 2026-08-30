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
# The voice host, which the subject reads for its loader contract rather than
# disassembling.
: >"${repo}/bazel-out/bin/reachy_host"

# The fetched shared object, at the path the stub's `info execution_root` and
# `cquery` answers put it — outside the repo, which is the whole difference
# between it and the two above.
onnx_dir="${work}/fixtures/execroot/external/onnxruntime_linux_aarch64/lib"
mkdir -p -- "$onnx_dir"
: >"${onnx_dir}/libonnxruntime.so.1"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# The bazel stub: two subcommands the subject uses. `cquery` answers the
# deployables filegroup with the listing shape the real one produces — the
# outputs plus the sources that came along with them, so the subject's basename
# matching has something to get wrong — and the ONNX Runtime target with the one
# execroot-relative path into the fetched repository the real one prints.
# `info execution_root` says where that path is rooted.
#
# The ONNX Runtime target is an alias over a `select()` on the target CPU, so
# the real cquery answers it differently depending on whether `--config=device`
# is on the command line. The stub answers the same way — aarch64 with the flag,
# x86_64 without it — because a query that lost the flag is the one regression
# on this subject that would otherwise leave every case here green.
export CQUERY_STATUS=""
export CQUERY_DROP=""
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
case $1 in
cquery)
	[ -z "${CQUERY_STATUS:-}" ] || exit "$CQUERY_STATUS"
	device=""
	for arg in "$@"; do
		[ "$arg" != --config=device ] || device=1
	done
	for arg in "$@"; do
		[ "$arg" = //bazel/third_party/onnxruntime:shared_object ] || continue
		if [ "${CQUERY_DROP:-}" != libonnxruntime.so.1 ]; then
			if [ -n "$device" ]; then
				echo external/onnxruntime_linux_aarch64/lib/libonnxruntime.so.1
			else
				echo external/onnxruntime_linux_x64/lib/libonnxruntime.so.1
			fi
		fi
		exit 0
	done
	echo bazel-out/bin/reachy_motord
	[ "${CQUERY_DROP:-}" = reachy_host ] || echo bazel-out/bin/reachy_host
	[ "${CQUERY_DROP:-}" = simplelaunch ] || echo bazel-out/bin/simplelaunch
	[ "${CQUERY_DROP:-}" = robot_clk_exe ] || echo bazel-out/bin/robot_clk_exe
	echo cogs/robot.clk
	;;
info)
	[ -z "${INFO_STATUS:-}" ] || exit "$INFO_STATUS"
	for arg in "$@"; do
		case $arg in
		execution_root) echo "${FIXTURES}/execroot"; exit 0 ;;
		esac
	done
	echo "${FIXTURES}/output_base"
	;;
*) echo "unstubbed subcommand $1" >&2; exit 1 ;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/bazel"

# The disassembler stub: prints the fixture named for the binary it was asked
# about, so a case chooses what the subject sees by writing that file. Exits
# non-zero when a case says to, which is the one failure mode a real objdump has
# that the subject has to report rather than read past.
#
# The two ONNX Runtime archives hold a file of the same name, so the x86_64 one
# gets its own fixture suffix: a case about which archive was resolved needs the
# two to disassemble differently, which in reality is the whole difference
# between them.
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
	name=$(basename -- "$arg")
	case "$arg" in
	*/onnxruntime_linux_x64/*) name="${name}.x64" ;;
	esac
	cat -- "${FIXTURES}/${name}"
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

# The voice host's dynamic section, as llvm-objdump -p prints one: a runpath
# whose first entry is the build tree's own -- which is what Bazel writes and
# what is nowhere on a unit -- and `$ORIGIN` appended after it, then the NEEDED
# entries. A case chooses what the subject sees by passing the two lines it
# wants; the default is the contract holding.
host_fixture() {
	# shellcheck disable=SC2016 # `$ORIGIN` is the loader's syntax, written out
	local runpath=${1-'$ORIGIN/../../_solib_local/onnxruntime:$ORIGIN'}
	local needed=${2-libonnxruntime.so.1}
	{
		header reachy_host
		printf 'Dynamic Section:\n'
		[ -z "$runpath" ] || printf '  RUNPATH      %s\n' "$runpath"
		[ -z "$needed" ] || printf '  NEEDED       %s\n' "$needed"
		printf '  NEEDED       libc.so.6\n'
	} >"${FIXTURES}/reachy_host"
}

# The green set: nothing but the guarded helpers in any of the three, and a
# voice host whose loader contract holds.
green() {
	fixture simplelaunch
	fixture robot_clk_exe
	fixture libonnxruntime.so.1
	host_fixture
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
	out=$(cd -- "$work" && "${1:-$subject}" 2>&1)
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
assert_contains "and the prebuilt runtime, which nobody here compiles" \
	"$(output_of "$result")" "libonnxruntime.so.1: no unguarded ARMv8.1 atomics."

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
for tool in bash env cat basename dirname awk sed tail head wc tr grep; do
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

# The prebuilt runtime carrying a straight-line LSE atomic. Its remedy is not
# the other two's: nothing in this tree chose its -march, so the way out is the
# pinned archive rather than a compile flag.
green
fixture libonnxruntime.so.1 \
	'0000000000401000 <OrtGetApiBase>:' \
	'  401000:      	casal	x0, x1, [x2]' \
	'  401004:      	ret'
result=$(run)
assert_status "an unguarded LSE atomic in the prebuilt fails" 1 "$(status_of "$result")"
assert_contains "naming the object" "$(output_of "$result")" \
	"libonnxruntime.so.1 carries instructions this unit's CPU does not implement"
assert_contains "with the remedy that fits a prebuilt" "$(output_of "$result")" \
	"Pin an ONNX Runtime release whose aarch64 build is not"
assert_lacks "and not the one that does not" "$(output_of "$result")" \
	"is still last on the compile"

# A build that names no shared object: the target was renamed or the archive
# stopped carrying the SONAME copy. A refusal, like a missing deployable.
green
CQUERY_DROP=libonnxruntime.so.1
result=$(run)
CQUERY_DROP=""
assert_status "a missing shared object is refused" 1 "$(status_of "$result")"
assert_contains "naming the query that answered nothing" "$(output_of "$result")" \
	"bazel named no output file for //bazel/third_party/onnxruntime:shared_object"

# The path the build names and the file system does not hold. The object lives
# outside the repo, so this is the branch a moved or unfetched external
# repository lands on.
green
mv -- "${onnx_dir}/libonnxruntime.so.1" "${onnx_dir}/moved"
result=$(run)
mv -- "${onnx_dir}/moved" "${onnx_dir}/libonnxruntime.so.1"
assert_status "an unfetched shared object is refused" 1 "$(status_of "$result")"
assert_contains "saying which build to run" "$(output_of "$result")" \
	"Run 'make check-device' first"

# bazel unable to say where its execution root is: the one path the subject
# cannot derive itself.
green
export INFO_STATUS=1
result=$(run)
unset INFO_STATUS
assert_status "a failed info is refused" 1 "$(status_of "$result")"
assert_contains "naming what could not be found" "$(output_of "$result")" \
	"cannot say where its execution root is"

# bazel itself failing.
green
CQUERY_STATUS=1
result=$(run)
CQUERY_STATUS=""
assert_status "a failed cquery is refused" 1 "$(status_of "$result")"
assert_contains "naming the query" "$(output_of "$result")" \
	"bazel cannot name the outputs of //bazel/platform:device_deployables"

# ---------------------------------------------------------------------------
# The configuration the shared object is resolved in
# ---------------------------------------------------------------------------

# `//bazel/third_party/onnxruntime:shared_object` is an alias over a `select()`
# on the target CPU, so which ELF the cquery names is decided entirely by
# `--config=device` being on the command line — unlike the deployables, whose
# paths merely move. A regression that queried it in the host configuration
# would hand the sweep the x86_64 runtime, which is not the object the unit
# loads and is not built for the CPU under test.
#
# The subject with that flag removed is the regression, run as a copy so the
# real one is untouched. What must not happen is a clean verdict.
x64_dir="${work}/fixtures/execroot/external/onnxruntime_linux_x64/lib"
mkdir -p -- "$x64_dir"
: >"${x64_dir}/libonnxruntime.so.1"

green
{
	header libonnxruntime.so.1 elf64-x86-64
	printf '  401000:      	retq\n'
} >"${FIXTURES}/libonnxruntime.so.1.x64"

hostwise="${repo}/tools/assert-device-isa-hostwise.sh"
sed 's/^build_flags=(--config=device)$/build_flags=()/' "$subject" >"$hostwise"
chmod 0755 -- "$hostwise"
grep -q '^build_flags=()$' "$hostwise" ||
	fail "the flag this case removes is no longer spelled that way in the subject"
result=$(run "$hostwise")
assert_status "querying the shared object without --config=device is refused" 1 \
	"$(status_of "$result")"
assert_contains "because what came back is the wrong architecture" \
	"$(output_of "$result")" "is not an aarch64 binary, so this checked the wrong build"

# ---------------------------------------------------------------------------
# The voice host's loader contract
# ---------------------------------------------------------------------------
#
# The one payload invariant that decides whether the binary starts at all, and
# the one nothing else in the tree asks the binary about: three files state it
# in prose and the payload is laid out around it. What it costs when it breaks
# is a host that dies at exec with a loader message, on a unit, narrating
# nothing -- so the red cases are what this section is for.

green
result=$(run)
assert_status "a host whose loader contract holds passes" 0 "$(status_of "$result")"
assert_contains "and the verdict says what it found" "$(output_of "$result")" \
	"reachy_host: NEEDED libonnxruntime.so.1, runpath carries \$ORIGIN"

# Bazel's own runpath entry is a path inside the build tree and resolves to
# nothing on a unit, so `$ORIGIN` alone lost from the list is the whole defect:
# the value still looks populated.
green
# shellcheck disable=SC2016 # the loader's syntax again
host_fixture '$ORIGIN/../../_solib_local/onnxruntime'
result=$(run)
assert_status "a runpath without \$ORIGIN is refused" 1 "$(status_of "$result")"
assert_contains "naming what the runpath does say" "$(output_of "$result")" \
	"_solib_local/onnxruntime"
assert_contains "and where the flag that writes it lives" "$(output_of "$result")" \
	"crates/reachy-host/BUILD.bazel"

# A host with no runpath at all: the same failure, and the message has to be
# about a runpath rather than about an empty variable.
green
host_fixture ''
result=$(run)
assert_status "a host with no runpath at all is refused" 1 "$(status_of "$result")"
assert_contains "saying it has none" "$(output_of "$result")" "it reads 'nothing'"

# A host that resolves ONNX Runtime from somewhere else -- a static link, or a
# differently named soname -- is not the binary this payload is staged for.
green
# shellcheck disable=SC2016 # the loader's syntax again
host_fixture '$ORIGIN' ''
result=$(run)
assert_status "a host with no NEEDED for the shared object is refused" 1 \
	"$(status_of "$result")"
assert_contains "naming the entry that is missing" "$(output_of "$result")" \
	"carries no NEEDED entry for libonnxruntime.so.1"

# Nothing dynamic at all reads as a file the tool could not make sense of, which
# must not pass as a contract that holds.
green
{
	header reachy_host
	printf 'Program Header:\n'
} >"${FIXTURES}/reachy_host"
result=$(run)
assert_status "a host with no dynamic section is refused" 1 "$(status_of "$result")"
assert_contains "saying it links nothing dynamically" "$(output_of "$result")" \
	"has no dynamic section"

# The host is read out of the same filegroup as everything else here, so a
# rename upstream is a refusal rather than a check that silently stopped
# happening.
green
CQUERY_DROP=reachy_host
result=$(run)
CQUERY_DROP=""
assert_status "a host missing from the build is refused" 1 "$(status_of "$result")"
assert_contains "naming what is missing" "$(output_of "$result")" \
	"the build emits no reachy_host"

tally
