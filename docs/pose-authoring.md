# Authoring a pose — from a recorded hold to a named asset

What this document is: the procedure for turning a hold on the de-torqued head
into a committed pose document, and for binding the poses roles use. The bench
and deploy mechanics — what `make pose-record` provisions, what `make
speech-run` pushes: `docs/bench-runbook.md`. Safety: `docs/fault-management.md`.

A pose is a whole base configuration — head, body yaw, both antennas — that the
mover moves to over a stated time. The documents live under `cogs/poses/`, one
`<name>.textproto` each; `cogs/pose_library.textproto` and
`cogs/library.names.json` are emitted from them and are never hand-edited.

A clip sequence must keep one provenance kind per channel after flattening. A
channel is either driven by posed clips, whose samples target their declared
base, or by unposed overlays; masked clips do not vote for channels they do not
drive. Mixing the two is refused at load time.

## Authoring a clip

    bazel run //cogs:pose_session_report -- <records> --out <dir> \
        --extract S0NN --name NAME --channels ...

This extractor always subtracts `neutral` and writes an unposed document with no
`base`. An overlay keeps that output. A posed extraction must add
`"base": "neutral"` before `make library-config`. Changing only the `base`
string to another pose is invalid: this tool did not express the frames over
that pose, and the load-time screen cannot infer the recording's provenance. A
clip over any non-neutral base must first have its frame deltas re-expressed
over that base by the authoring process.

Any clip carrying a pose must be posed, or playing it over that pose applies the
pose twice. A flattened sequence must retain the already-documented
one-provenance-per-channel rule.

Antennas in posed frames are directions. When the channel comes in, playback
takes the short arc from the current commanded antenna setpoint and keeps that
whole-turn representative until the channel fades out; neither the authored
direction's nor the standing base's scalar representative changes that choice.
Unposed antenna deltas remain relative and ride the base as authored.

`make library-config` screens authored frames (over neutral for an overlay, over
the declared anchor for a posed clip), regenerates the asset and sidecar, and a
script is rehearsed with `make motion-host-script SCRIPT=...` before
`make motion-script SCRIPT=...` on a unit.

## The five steps

**1. Record.** `make pose-record`, hands on the de-torqued head; hold each pose
still long enough for the segmenter to see a hold, and say what it is. `make
pose-fetch` brings the session home under `.local/pose-sessions/`.

**2. Extract a draft.**

    bazel run //cogs:pose_session_report -- <session> --out <dir> \
        --extract S00N --as-pose --name <name>

For a derived draft, use `--lift-mm <mm>` for a finite positive lift or
`--scale <k>` for a finite positive scale toward neutral, and always state a
positive whole-number `--duration-ms <ms>`. The two derivations are mutually
exclusive and valid only with `--as-pose`. A lift changes only head Z; a scale
changes only head translation and rotation. Body yaw and antenna directions
remain recorded content until the author makes an explicit content choice.

The four segmenter flags (`--speed-window-ms`, `--still-rad-s`,
`--moving-rad-s`, `--min-still-ms`) tune what counts as a hold; re-running the
report over the fetched session is cheaper than re-recording, and the values
used travel in `session.json`, because a segment id means nothing without the
configuration that produced it. `--as-pose` refuses a moving segment: a pose is
a hold.

The command writes `pose-S00N.textproto` beside the session document and prints
the static envelope verdict followed by a directed minimum-jerk transition
verdict against the committed library. **Read both verdicts.** A still that
fails the envelope or whose path fails the transition gate is not a pose this
machine can be commanded to; the answer is another recording, a derived draft
(`--lift-mm` for a path-only problem or `--scale` for an endpoint, window or
floor problem), or a reviewed floor change using the derivation in
`EnvelopeConfig::default`. It is never an edited head figure in the document.

For a scale derivation, sweep downward on the 0.01 grid until both verdicts
pass; retain the largest passing factor. Author and emit a batch of new poses
one at a time, regenerating the library after each one so the next draft sees
the library as it stands. A paced draft is checked at 0 ms, every 20 ms before
its endpoint, and the exact endpoint in both directions to every committed
pose.

Then read the draft: `body_yaw` is whatever a hand left the base at, so set it
to `0.0` unless the pose turns the base; the antennas are already reduced to
directions; a derived draft already carries its required pace. Move the file to
`cogs/poses/<name>.textproto`.

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
