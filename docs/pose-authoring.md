# Authoring a pose — from a recorded hold to a named asset

What this document is: the procedure for turning a hold on the de-torqued head
into a committed pose document, and for binding the poses roles use. The bench
and deploy mechanics — what `make pose-record` provisions, what `make
speech-run` pushes: `docs/bench-runbook.md`. Safety: `docs/fault-management.md`.

A pose is a whole base configuration — head, body yaw, both antennas — that the
mover moves to over a stated time. The documents live under `cogs/poses/`, one
`<name>.textproto` each; `cogs/pose_library.textproto` and
`cogs/library.names.json` are emitted from them and are never hand-edited.

## The five steps

**1. Record.** `make pose-record`, hands on the de-torqued head; hold each pose
still long enough for the segmenter to see a hold, and say what it is. `make
pose-fetch` brings the session home under `.local/pose-sessions/`.

**2. Extract a draft.**

    bazel run //cogs:pose_session_report -- <session> --out <dir> \
        --extract S00N --as-pose --name <name>

The four segmenter flags (`--speed-window-ms`, `--still-rad-s`,
`--moving-rad-s`, `--min-still-ms`) tune what counts as a hold; re-running the
report over the fetched session is cheaper than re-recording, and the values
used travel in `session.json`, because a segment id means nothing without the
configuration that produced it. `--as-pose` refuses a moving segment: a pose is
a hold.

The command writes `pose-S00N.textproto` beside the session document and prints
the envelope verdict for the pose it drafted — per-leg toggle margins, the
smallest against the clearance floor, the cone and relative yaw, and on a
refusal the violations. **Read the verdict.** A still that fails the envelope is
not a pose this machine can be commanded to; the fix is another recording, or —
if the recording is the machine's real rest — a reviewed change to the floor in
`crates/reachy-kin/src/envelope.rs`. It is never an edited number in the
document.

Then read the draft: `body_yaw` is whatever a hand left the base at, so set it
to `0.0` unless the pose turns the base; the antennas are already reduced to
directions; `duration_ms` is `0` and must be set, because the loader refuses a
pose with no pace. Move the file to `cogs/poses/<name>.textproto`.

A pose that is not a recording — `neutral` is one — is written by hand to the
same schema. Either way the `description` says where the figures came from.

**3. Emit.** `make library-config` regenerates `cogs/pose_library.textproto`
and `cogs/library.names.json` from the documents. Commit the three together;
`make check` refuses a stale emit, a document the loader refuses, a duplicate
name, a document whose `name` is not its file stem, and a library with no pose
named `stow`.

**4. Bind roles.** `presence_wake_pose` and `presence_turn_pose` in the speech
configuration name the pose that means *listening* and the pose that means
*engaged*. The stow is not bound anywhere: it is the document named `stow` —
where the machine rests, what every schedule ends at, what the disarm sequence
judges folded against. Making a different recording the stow is renaming a
document; the one it replaces keeps a name of its own or goes.

Where the head's poses are, and how fast the head goes to one when nobody says,
are authored here — the documents' figures and their `duration_ms`. How fast the
*scripter's* raise, turn and stow go is a speech-configuration key:
`presence_wake_move_ms`, `presence_turn_move_ms` and `presence_stow_move_ms`
(brenn-pod's `docs/runbooks/reachy-end-to-end.md`), each absent by default, and
absent means the library's own pace. Those keys are a command the scripter puts
on the wire, not a second copy of a duration held here. The bench harness and
every stow the machine plans for itself state no pace and take the library's.
The `stow` document's pace is bounded by the fault ladder's clock:
`docs/fault-management.md`, "One clock", which is where that rule is stated.

**5. Deploy.** `make speech-run` builds, pushes the payload into the unit's
tmpfs and starts the processes. **Every change needs this**: the host reads the
sidecar at start and the box binds the library at start; nothing hot-reloads.
`reachy_host --check` runs in the preflight and refuses a role that names a pose
the sidecar lacks.

## One emit reaches a unit, whole

The library and the sidecar reach a unit only as members of one staged payload,
rsync'd whole, and neither is on the experiment overlay's allowlist
(`docs/servo-tuning.md`), so a unit cannot hold a sidecar from one emit and a
library from another. That, the drift test and the single generator are the
identity guarantee between the two files; there is no runtime check of it.

The price is that the *library's* pace is not retunable on the unit — changing
it is an authoring edit and a redeploy. The scripter's three
`presence_*_move_ms` keys are retunable, by the `speech.toml` path step 4 names.
