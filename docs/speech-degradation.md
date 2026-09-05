# Speech degradation — reading a run, and comparing two

## The symptom

The robot wakes, the head rises, and nothing follows. The utterance was heard
and transcribed; the transcript was then declined by the STT-confidence gate as
a likely hallucination, so it never reached the brain. On this unit the audio
degrades as a conversation goes on — the microphone board's adaptive
post-processing learns the servo noise and suppresses speech with it — and
recovers after a few minutes with no wake: a first utterance after a quiet
interval reads clean, later ones degraded.

## Reading a run

`make speech-run` fetches the records, `<run>.console` and `<run>.audio`, then
prints one line per turn:

    turn #2 — wake 0.76 → "Test two." no_speech=0.30 logprob=-1.05 → declined
      (low_confidence); clip turn-02.wav [173696–219072); auto beam 2.15±0.34 rad

`no_speech` is the reading. Clean utterances on this unit sit at `0.01–0.09`;
degraded ones at `0.23–0.54`; the gate declines above `0.2`. The summary's
range line — `no_speech: dispatched 0.075; declined 0.42–0.50` — is what a
session is judged on. Declined turns are printed again under a line counting the transcripts with
words in them that went unanswered; the verdict stays green, because a wake
with nothing said after it is rightly declined.

## Listening to a turn

The turn line names its `.wav` under `<run>.turns/`. The clip is the whole
carve, wake word included. Beside it, `turn-NN.command.wav` holds the same span
from the wake-trim boundary, where the boundary was known. The line states both
offsets: `STT boundary +N s` is where the wake word ends, `sent from +M s` is
where transcription actually began. The two are equal under `[stt] wake_word =
"trim"`, and that is the run in which the second file is what the recogniser was
given; under `"keep"` the second offset is zero and the recogniser was given the
whole first file. `clip not written (…)` says why there is no file, and `no
turn-NN.command.wav (…)` says why only the second one is missing.

`held up to N s for the command` on a turn line says the wake word arrived
alone and the listener kept the wake open for the command that followed. A wake
that was held and never answered is counted on its own line.

To ask which of the two clips reads better, hand both to the recogniser:

```
bazel run //crates/reachy-host:stt_compare -- \
    --speech-config <the speech.toml the run used> <run>.turns
```

One line per turn with both transcripts and their confidence figures, and a
count of the turns that read differently once case and punctuation are folded
away. It dials the recogniser the configuration names, so it is the one tool
here that is not offline; a turn with no second clip is listed and not asked.

## The chip

The pod's `pod_0.log` carries a startup line and a repeating state line: both
output routings, the ASR-output switch and gain, whether the echo canceller
converged, and the post-processing parameters. A reboot line says whether the board was seen leaving the bus; one that never
left may not have rebooted, and may still hold its adaptive state. `channel=0 (routing refused)` on
the startup line means the firmware would not take the ASR-output routing and
the run used the post-processed channel — the pipeline as it was before this
routing existed.

## Comparing two sessions

Four wake-and-command turns each, replies played, two minutes of silence
first. `CHANNEL=0` in the pod's local tuning fragment forces the post-processed
channel; removing the line gives the ASR output. Expect the first session to
degrade after turn 1 and the second to dispatch all four under `0.1`, with no
barge-ins and the echo canceller converged after the first reply.
