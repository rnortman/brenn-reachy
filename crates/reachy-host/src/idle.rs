//! The idle loop's decisions: which clip the head dances next, and when.
//!
//! A third source of intent beside the scripter and the bus. With nobody
//! speaking, the loop dances the playlist: it opens a resting machine with one
//! clip and replaces that clip with the next just before it ends, so a session
//! it keeps fed never runs down to its own stow. When speech takes the head the
//! loop steps back, and it takes the head again where speech says it is done.
//!
//! Pure: `(now, story rows, speech bodies, edge outcomes) → (script, due)`.
//! `follow` owns the clock and the sockets, offers what [`Idle::poll`] returns
//! through the same gate every other body goes through, and tells the loop what
//! the edge answered. Nothing here reads a clock, opens a socket or sleeps.
//!
//! Who owns the head is said once, in the wire's vocabulary. A script that is
//! not a bare stow takes the head for speech once the edge has accepted it; a
//! script whose last step is a future `stow` says when speech is done with it,
//! and the `resting` that ends the session running it hands it back; a body
//! the session holds across a `resting` keeps the head through that one. The
//! loop owns it otherwise. A body the edge refused never ran and changes
//! nothing. A bare stow from a sender that no longer owns the head is stale,
//! and the loop has it dropped rather than offered: it would fold a head that is dancing.
//!
//! The loop reads three row kinds — `phase_changed`, `session_ended` and
//! `script_refused` — and waits on none of them. The story can lose rows, and a
//! loop that waited for one (a confirm row, a `script_replaced`) would stop for
//! good on one overrun; a lost row here costs one backoff step or is corrected
//! by the next phase row or a refusal.
//!
//! A refusal of the loop's own script halts it until the next phase row. Its
//! scripts are fixed shapes over a checked playlist, so a refusal is a race or
//! a bug, and answering it with the next clip would retry a failed send with a
//! perturbed input.
//!
//! A loop that has heard no story at all [`NO_STORY_GRACE_MS`] after its first
//! poll offers one opening, once per process run. The control process
//! publishes its story only when it adds a row, so a host bound after the boot
//! row went out hears nothing until the session answers something, and the
//! answer carries the whole ring, boot row included. The opening goes through
//! the same gate and intake as any other script; a refusal halts the loop as
//! any refusal does, and nothing sends a second.
//!
//! The loop numbers its scripts from the wall clock, above the last speech
//! body the edge accepted, on the one sequence gate every sender shares. A
//! wall clock stepped backwards leaves speech's bodies refused stale until
//! the clock passes the loop's last number — the bounded, self-healing
//! residual `SeqSource` already accepts across a restart — and the edge's
//! staleness alert, which the loop's own acceptances do not reset, says
//! so.

use std::path::{Path, PathBuf};

use clockwork_rs::SyncTime;
use motion_proto::{Action, Base, MotionScript, Play, STOW_POSE, SeqSource, Step};
use reachy_edge::Origin;
use reachy_edge::names::{MotionEntry, MotionTable};
use reachy_edge::narrate::{edge_line_with, origin_word, row_says};
use reachy_edge::run::Surface;
use reachy_edge::story::Update;
use serde::Deserialize;
use serde_json::{Value, json};

use brenn_reachy__cogs__session_clk_rs::{SessionPhase, SessionPhaseWire};
use brenn_reachy__motion__reports_clk_rs::ReportKind;
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

use crate::words::{
    IDLE_BACKOFF, IDLE_DROPPED_STOW, IDLE_NO_STORY, IDLE_OPENED, IDLE_PARKED, IDLE_REFUSED,
    IDLE_REPLACED, IDLE_RESUMED, IDLE_SUSPENDED,
};

/// The pose every loop script opens on: a resting head is raised to it, and a
/// dancing one is steered back to it under the next clip.
/// `the_loop_is_paced_as_the_committed_library_paces_it` holds it to the
/// committed library's poses.
pub const NEUTRAL_POSE: &str = "neutral";

/// Where an opening script's clip starts: the engagement's allowance plus the
/// raise to [`NEUTRAL_POSE`], so the clip blends in from a head that arrived.
pub const OPEN_PLAY_MS: u64 = 2_500;

/// Where a replacement script's clip starts: one mover period after arrival.
pub const SEAM_PLAY_MS: u64 = 20;

/// How far ahead of the outgoing clip's end the replacement is sent, so it
/// reaches the mover before that clip's ramp-out begins.
pub const SEAM_LEAD_MS: u64 = 60;

/// How long after a clip's blend-out its script stows, which is how long a
/// late replacement has before the session winds itself down.
pub const STOW_MARGIN_MS: u64 = 5_000;

/// What a script allows its closing stow before its timeout.
/// `the_loop_is_paced_as_the_committed_library_paces_it`, in the idle playlist
/// test, holds it to the committed library's stow.
pub const STOW_DURATION_MS: u64 = 2_000;

/// How long a session must stay `active` for the backoff ladder to start over.
pub const LADDER_RESET_S: u64 = 60;

/// The wait after the first fault ending in a row.
pub const BACKOFF_FIRST_S: u64 = 15;

/// The longest wait the ladder climbs to.
pub const BACKOFF_CAP_S: u64 = 120;

/// How long after its first poll the loop waits for any story before saying
/// it has heard none and offering its one opening.
pub const NO_STORY_GRACE_MS: u64 = 10_000;

/// Name prefixes a playlist may not hold: probes and bench motions are not
/// content anyone watched as a dance.
pub const EXCLUDED_PREFIXES: [&str; 2] = ["probe/", "bench/"];

/// The motions the loop picks from, checked against the deployed name table.
///
/// File order is kept. At least two entries, no duplicates, none excluded.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Playlist {
    entries: Vec<(String, MotionEntry)>,
}

/// The playlist file's one shape. A misspelt field is refused rather than read
/// as an empty list.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PlaylistFile {
    playlist: Vec<String>,
}

impl Playlist {
    /// The playlist `text` states, resolved through `table`.
    ///
    /// # Errors
    ///
    /// [`IdleError::Malformed`] for text that is not `{"playlist": [..]}`; for
    /// the first name that is excluded, repeated or not in `table`,
    /// [`IdleError::Excluded`], [`IdleError::Duplicate`] or
    /// [`IdleError::UnknownMotion`]; [`IdleError::TooFew`] for fewer than two.
    pub fn parse(text: &str, table: &MotionTable) -> Result<Self, IdleError> {
        let file: PlaylistFile =
            serde_json::from_str(text).map_err(|error| IdleError::Malformed {
                detail: error.to_string(),
            })?;
        let mut entries: Vec<(String, MotionEntry)> = Vec::with_capacity(file.playlist.len());
        for name in file.playlist {
            if EXCLUDED_PREFIXES
                .iter()
                .any(|prefix| name.starts_with(prefix))
            {
                return Err(IdleError::Excluded { name });
            }
            if entries.iter().any(|(seen, _)| *seen == name) {
                return Err(IdleError::Duplicate { name });
            }
            let Some(entry) = table.resolve(&name) else {
                return Err(IdleError::UnknownMotion { name });
            };
            entries.push((name, entry));
        }
        if entries.len() < 2 {
            return Err(IdleError::TooFew {
                count: entries.len(),
            });
        }
        Ok(Self { entries })
    }

    /// How many motions the playlist holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the playlist holds none, which a parsed one never does.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The motions, in file order.
    pub fn entries(&self) -> impl Iterator<Item = (&str, MotionEntry)> {
        self.entries
            .iter()
            .map(|(name, entry)| (name.as_str(), *entry))
    }
}

/// Why a playlist was not loaded.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum IdleError {
    /// The file could not be read.
    #[error("the playlist at {} could not be read: {detail}", path.display())]
    Read {
        /// Where it was looked for.
        path: PathBuf,
        /// What the read said.
        detail: String,
    },
    /// The text is not `{"playlist": [string, …]}`.
    #[error("the playlist is not {{\"playlist\": [name, …]}}: {detail}")]
    Malformed {
        /// What the parser said.
        detail: String,
    },
    /// A name the deployed library does not hold.
    #[error("the playlist names {name}, which the name table does not hold")]
    UnknownMotion {
        /// The name.
        name: String,
    },
    /// A probe or bench motion.
    #[error("the playlist names {name}, which is a probe or bench motion and never a dance")]
    Excluded {
        /// The name.
        name: String,
    },
    /// A name listed twice.
    #[error("the playlist names {name} twice")]
    Duplicate {
        /// The name.
        name: String,
    },
    /// Fewer than two motions: the loop never plays one twice in a row.
    #[error("the playlist holds {count} motion(s), and the loop needs two to alternate")]
    TooFew {
        /// How many it holds.
        count: usize,
    },
}

/// What [`Idle::speech`] says about a speech body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Offer it to the edge.
    Offer,
    /// Do not offer it: a stale stow from a sender that no longer owns the head.
    Drop,
}

/// What one [`Idle::poll`] asks of its caller.
#[derive(Debug, Default, PartialEq)]
pub struct Poll {
    /// A script to offer, at most one.
    pub send: Option<MotionScript>,
    /// The next instant anything the loop waits on falls due, strictly after
    /// the poll's `now`.
    pub due: Option<SyncTime>,
}

/// Who owns the head.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Owner {
    Loop,
    Speech,
}

/// The loop's script the session is running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Standing {
    script_id: u32,
    motion_id: u16,
    due: SyncTime,
}

/// Which offsets a script used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shape {
    Opening,
    Replacement,
}

/// What the last poll returned, until the edge's answer resolves it.
#[derive(Clone, Debug)]
struct Pending {
    index: usize,
    shape: Shape,
    resume: bool,
}

/// The idle loop's state.
///
/// Not `Clone`: it holds a sequence source, and one sender has one.
#[derive(Debug)]
pub struct Idle {
    playlist: Playlist,
    pod: String,
    /// The phase the last phase row entered; `None` before any, or for a
    /// number this build does not name.
    phase: Option<SessionPhase>,
    standing: Option<Standing>,
    owner: Owner,
    /// The last speech body the edge accepted arrived while the session was
    /// between sessions (`starting`, `engaging` or `stopping`). The session
    /// holds such a body and runs it after the next phase change, so the
    /// `resting` that follows is not the end of speech's hold on the head.
    speech_held: bool,
    /// When the head comes back to the loop: speech's stow instant, or the end
    /// of a backoff.
    resume_at: Option<SyncTime>,
    /// A `session_ended` row seen since the last phase row.
    clean_end: bool,
    active_since: Option<SyncTime>,
    /// A refusal of the loop's script in this phase.
    halted: bool,
    consecutive_faults: u32,
    last_motion: Option<u16>,
    parked: bool,
    pending: Option<Pending>,
    seq: SeqSource,
    /// The sequence number of the last speech body the edge accepted. The
    /// edge keeps one mark for every sender, and a speech body accepted on
    /// the pass that resumes the loop can carry the loop's own millisecond,
    /// so the loop numbers above it.
    speech_mark: Option<u64>,
    rng: u64,
    /// A story datagram has reached the loop since the process started.
    story_seen: bool,
    first_poll: Option<SyncTime>,
    no_story_said: bool,
}

/// The xorshift state a zero seed is replaced with; zero is its fixed point.
const ZERO_SEED: u64 = 0x9E37_79B9_7F4A_7C15;

impl Idle {
    /// The playlist at `path`, resolved through `table`.
    ///
    /// # Errors
    ///
    /// [`IdleError::Read`] when the file cannot be read, and whatever
    /// [`Playlist::parse`] refuses.
    pub fn load(path: &Path, table: &MotionTable) -> Result<Playlist, IdleError> {
        let text = std::fs::read_to_string(path).map_err(|error| IdleError::Read {
            path: path.to_path_buf(),
            detail: error.to_string(),
        })?;
        Playlist::parse(&text, table)
    }

    /// A loop over `playlist`, addressing `pod`, picking with `seed`.
    #[must_use]
    pub fn new(playlist: Playlist, pod: &str, seed: u64) -> Self {
        Self {
            playlist,
            pod: pod.to_owned(),
            phase: None,
            standing: None,
            owner: Owner::Loop,
            speech_held: false,
            resume_at: None,
            clean_end: false,
            active_since: None,
            halted: false,
            consecutive_faults: 0,
            last_motion: None,
            parked: false,
            pending: None,
            seq: SeqSource::new(),
            speech_mark: None,
            rng: if seed == 0 { ZERO_SEED } else { seed },
            story_seen: false,
            first_poll: None,
            no_story_said: false,
        }
    }

    /// Everything back to [`Self::new`]'s values but the playlist, the pod,
    /// the sequence source, the speech mark and the rng, and what the loop
    /// has heard since the process started. A control process that restarted is a
    /// new machine run: it commissions and rests, so `parked` clears too.
    fn reset(&mut self) {
        self.phase = None;
        self.standing = None;
        self.owner = Owner::Loop;
        self.speech_held = false;
        self.resume_at = None;
        self.clean_end = false;
        self.active_since = None;
        self.halted = false;
        self.consecutive_faults = 0;
        self.last_motion = None;
        self.parked = false;
        self.pending = None;
    }

    /// What one story datagram added. Each row's own time is the loop's clock
    /// for what that row sets: the session and the host stamp from this
    /// machine's one wall clock. A count of lost rows is not read; the rules
    /// below are built so that a lost row never strands the loop.
    pub fn story(&mut self, update: &Update, surface: &mut impl Surface) {
        self.story_seen = true;
        if update.restarted {
            self.reset();
        }
        for row in &update.rows {
            match row.kind().to_known() {
                Some(ReportKind::SessionEnded) => self.clean_end = true,
                Some(ReportKind::PhaseChanged) => self.phase_changed(row, surface),
                Some(ReportKind::ScriptRefused) => self.script_refused(row, surface),
                _ => {}
            }
        }
    }

    fn phase_changed(&mut self, row: &TimelineEntryWire, surface: &mut impl Surface) {
        let held = std::mem::take(&mut self.speech_held);
        let at = row.time();
        let entered = phase_of(row.a());
        let left = phase_of(row.b());
        self.halted = false;
        self.phase = entered;
        self.active_since = (entered == Some(SessionPhase::Active)).then_some(at);
        if entered == Some(SessionPhase::Resting) {
            self.standing = None;
            // A speech body the session is holding runs after this `resting`,
            // so speech keeps the head and a closing keeps its resume instant;
            // the `resting` that ends that body's session hands the head back.
            if !held {
                self.owner = Owner::Loop;
            }
            if left == Some(SessionPhase::Starting) {
                // A machine that has just commissioned: no session ended, so
                // there is no ending to classify and the loop opens at once.
                if !held {
                    self.resume_at = None;
                }
            } else if self.clean_end {
                self.consecutive_faults = 0;
                if !held {
                    self.resume_at = None;
                }
            } else {
                self.consecutive_faults = self.consecutive_faults.saturating_add(1);
                let wait = backoff_s(self.consecutive_faults);
                self.resume_at = Some(plus_ms(at, wait.saturating_mul(1_000)));
                surface.say(edge_line_with(
                    IDLE_BACKOFF,
                    at,
                    &format!("the session ended on a fault; the loop opens again in {wait} s"),
                    &[
                        ("seconds", json!(wait)),
                        ("consecutive_faults", json!(self.consecutive_faults)),
                    ],
                ));
            }
        }
        self.clean_end = false;
        if entered == Some(SessionPhase::Parked) && !self.parked {
            self.parked = true;
            surface.say(edge_line_with(
                IDLE_PARKED,
                at,
                "the machine parked; the loop sends nothing until the control process restarts",
                &[],
            ));
        }
    }

    fn script_refused(&mut self, row: &TimelineEntryWire, surface: &mut impl Surface) {
        if !self.standing.is_some_and(|s| s.script_id == row.a()) {
            return;
        }
        self.standing = None;
        self.halted = true;
        surface.say(edge_line_with(
            IDLE_REFUSED,
            row.time(),
            &format!(
                "the session refused the loop's script ({}); the loop sends nothing until the next phase change",
                row_says(row)
            ),
            &[
                ("script_id", json!(row.a())),
                ("reason", json!("session")),
                ("refusal", json!(row.b())),
            ],
        ));
    }

    /// Whether a speech body dequeued at `at` from `origin` is offered. Reads
    /// the owner and changes nothing; the one line it writes is a dropped stow.
    pub fn speech(
        &self,
        body: &[u8],
        origin: Origin,
        at: SyncTime,
        surface: &mut impl Surface,
    ) -> Verdict {
        let Some(script) = decoded(body) else {
            return Verdict::Offer;
        };
        if !bare_stow(&script) || self.owner == Owner::Speech {
            return Verdict::Offer;
        }
        surface.say(edge_line_with(
            IDLE_DROPPED_STOW,
            at,
            "a stow from a sender that no longer owns the head was not offered; the loop is dancing",
            &[("origin", json!(origin_word(origin)))],
        ));
        Verdict::Drop
    }

    /// The edge accepted a speech body stamped `arrival`. The one place a
    /// speech body changes the loop's state.
    pub fn speech_accepted(&mut self, body: &[u8], arrival: SyncTime, surface: &mut impl Surface) {
        let Some(script) = decoded(body) else {
            return;
        };
        self.speech_mark = Some(script.seq());
        if bare_stow(&script) {
            return;
        }
        self.speech_held = matches!(
            self.phase,
            Some(SessionPhase::Starting | SessionPhase::Engaging | SessionPhase::Stopping)
        );
        if self.owner == Owner::Loop {
            surface.say(edge_line_with(
                IDLE_SUSPENDED,
                arrival,
                "speech took the head; the loop waits until speech is done with it",
                &[],
            ));
            self.owner = Owner::Speech;
        }
        self.standing = None;
        self.resume_at = match script.steps().last() {
            Some(Step {
                after_ms,
                action: Action::Base(Base::Pose { name, .. }),
            }) if name == STOW_POSE && *after_ms > 0 => {
                Some(minus_ms(plus_ms(arrival, *after_ms), SEAM_LEAD_MS))
            }
            _ => None,
        };
    }

    /// The edge accepted the script the last poll returned, as `script_id`,
    /// stamped `arrival`.
    pub fn sent(&mut self, script_id: u32, arrival: SyncTime, surface: &mut impl Surface) {
        let Some(pending) = self.pending.take() else {
            return;
        };
        let Some((name, entry)) = self.playlist.entries.get(pending.index) else {
            return;
        };
        let play = play_ms(pending.shape);
        self.standing = Some(Standing {
            script_id,
            motion_id: entry.motion_id,
            due: minus_ms(
                plus_ms(arrival, play.saturating_add(entry.window.duration_ms)),
                SEAM_LEAD_MS,
            ),
        });
        self.last_motion = Some(entry.motion_id);
        self.resume_at = None;
        if pending.resume {
            self.owner = Owner::Loop;
        }
        let (word, says) = match (pending.resume, pending.shape) {
            (true, Shape::Replacement) => (
                IDLE_RESUMED,
                format!("speech is done with the head; the loop resumes with {name}"),
            ),
            (_, Shape::Opening) => (IDLE_OPENED, format!("the loop opens a session with {name}")),
            (false, Shape::Replacement) => (
                IDLE_REPLACED,
                format!("the loop replaces the standing clip with {name}"),
            ),
        };
        surface.say(edge_line_with(
            word,
            arrival,
            &says,
            &[("motion", json!(name)), ("script_id", json!(script_id))],
        ));
    }

    /// The edge refused, at `at`, the script the last poll returned.
    pub fn refused(&mut self, at: SyncTime, surface: &mut impl Surface) {
        if self.pending.take().is_none() {
            return;
        }
        self.halted = true;
        self.standing = None;
        surface.say(edge_line_with(
            IDLE_REFUSED,
            at,
            "the edge refused the loop's script, and its own line beside this one says why; the loop sends nothing until the next phase change",
            &[("script_id", Value::Null), ("reason", json!("edge"))],
        ));
    }

    /// The edge accepted the script the last poll returned, as `script_id`,
    /// and it could not be sent to the control process. Nothing is running
    /// it, so nothing stands; the loop halts as it does on a refusal.
    pub fn unsent(&mut self, script_id: u32, at: SyncTime, surface: &mut impl Surface) {
        if self.pending.take().is_none() {
            return;
        }
        self.halted = true;
        self.standing = None;
        surface.say(edge_line_with(
            IDLE_REFUSED,
            at,
            "the loop's script could not be sent to the control process, and the line beside this one says why; the loop sends nothing until the next phase change",
            &[("script_id", json!(script_id)), ("reason", json!("unsent"))],
        ));
    }

    /// What the loop wants at `now`: a script to offer, and when to ask again.
    pub fn poll(&mut self, now: SyncTime, surface: &mut impl Surface) -> Poll {
        self.pending = None;
        let mut probe = false;
        if !self.story_seen && !self.no_story_said {
            let since = *self.first_poll.get_or_insert(now);
            if now >= plus_ms(since, NO_STORY_GRACE_MS) {
                self.no_story_said = true;
                probe = true;
                surface.say(edge_line_with(
                    IDLE_NO_STORY,
                    now,
                    &format!("no story from the control process in {} s; the loop offers one opening, and the session's answer to it carries the story it has not heard", NO_STORY_GRACE_MS / 1_000),
                    &[("seconds", json!(NO_STORY_GRACE_MS / 1_000))],
                ));
            }
        }
        if self.parked {
            return Poll::default();
        }
        if self.consecutive_faults > 0
            && self.phase == Some(SessionPhase::Active)
            && self
                .active_since
                .is_some_and(|since| now >= ladder_reset_at(since))
        {
            self.consecutive_faults = 0;
        }

        let mut send = None;
        let wanted = if probe {
            Some((Shape::Opening, false))
        } else {
            self.choice(now)
        };
        if let Some((shape, resume)) = wanted {
            let index = self.pick();
            match self.script_for(index, shape, now) {
                Ok(script) => {
                    self.pending = Some(Pending {
                        index,
                        shape,
                        resume,
                    });
                    send = Some(script);
                }
                Err(error) => {
                    self.halted = true;
                    self.standing = None;
                    surface.say(edge_line_with(
                        IDLE_REFUSED,
                        now,
                        "the loop could not build its own script; it sends nothing until the next phase change",
                        &[
                            ("script_id", Value::Null),
                            ("reason", json!("unbuildable")),
                            ("detail", json!(error.to_string())),
                        ],
                    ));
                }
            }
        }

        Poll {
            send,
            due: self.due(now),
        }
    }

    /// Which script, if any, the state asks for at `now`, and whether sending
    /// it hands the head back from speech.
    fn choice(&self, now: SyncTime) -> Option<(Shape, bool)> {
        if self.halted {
            return None;
        }
        let resumed = self.resume_at.is_some_and(|r| now >= r);
        let speech = self.owner == Owner::Speech;
        match self.phase {
            // At `resting` the story says nothing more until the session
            // engages, so an opening already sent is still standing and a
            // second would be sent on every poll until then.
            Some(SessionPhase::Resting) if self.standing.is_none() => {
                let turn = if speech {
                    resumed
                } else {
                    self.resume_at.is_none_or(|r| now >= r)
                };
                turn.then_some((Shape::Opening, speech))
            }
            Some(SessionPhase::Active) => {
                let turn = if speech {
                    resumed
                } else {
                    self.standing.is_some_and(|s| now >= s.due)
                };
                turn.then_some((Shape::Replacement, speech))
            }
            _ => None,
        }
    }

    /// The earliest deadline the loop waits on that is strictly after `now`.
    /// One that already passed without firing is waiting on a phase row, and
    /// returning it would have the caller spin.
    fn due(&self, now: SyncTime) -> Option<SyncTime> {
        let active = self.phase == Some(SessionPhase::Active);
        let standing = self
            .standing
            .filter(|_| active && self.owner == Owner::Loop && !self.halted)
            .map(|s| s.due);
        let resume = self.resume_at.filter(|_| {
            !self.halted
                && matches!(
                    self.phase,
                    Some(SessionPhase::Resting | SessionPhase::Active)
                )
        });
        let ladder = self
            .active_since
            .filter(|_| self.consecutive_faults > 0 && active)
            .map(ladder_reset_at);
        [standing, resume, ladder]
            .into_iter()
            .flatten()
            .filter(|&at| at > now)
            .min()
    }

    /// The next playlist index: uniform over every entry but the last motion
    /// sent, drawn with xorshift64*.
    fn pick(&mut self) -> usize {
        let mut x = self.rng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.rng = x;
        let out = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
        let candidates: Vec<usize> = self
            .playlist
            .entries
            .iter()
            .enumerate()
            .filter(|(_, (_, entry))| Some(entry.motion_id) != self.last_motion)
            .map(|(index, _)| index)
            .collect();
        // Never empty: `Playlist::parse` admits at least two distinct names, and
        // `MotionTable` refuses two names over one motion id, so excluding the
        // last motion leaves at least one.
        assert!(
            !candidates.is_empty(),
            "a parsed playlist always leaves a candidate"
        );
        let len = u64::try_from(candidates.len()).expect("a candidate count fits in u64");
        let at =
            usize::try_from(out % len).expect("an index below the candidate count fits in usize");
        candidates[at]
    }

    /// The script for playlist entry `index` in `shape`, numbered at `now`.
    fn script_for(
        &mut self,
        index: usize,
        shape: Shape,
        now: SyncTime,
    ) -> Result<MotionScript, motion_proto::ScriptError> {
        let (name, entry) = &self.playlist.entries[index];
        let play = play_ms(shape);
        let stow_at = play
            .saturating_add(entry.window.duration_ms)
            .saturating_add(entry.window.blend_out_ms)
            .saturating_add(STOW_MARGIN_MS);
        let steps = vec![
            Step::new(0, NEUTRAL_POSE),
            Step::play(play, Play::new(name.clone())),
            Step::new(stow_at, STOW_POSE),
        ];
        // The edge's mark is shared with speech, so the loop numbers above the
        // last speech body it accepted.
        let above = self.speech_mark.map_or(0, |mark| mark.saturating_add(1));
        let seq = self.seq.next(unix_ms(now).max(above));
        MotionScript::new(
            self.pod.clone(),
            seq,
            steps,
            stow_at.saturating_add(STOW_DURATION_MS),
        )
    }
}

/// The phase a row's number names, or `None` for one this build does not.
fn phase_of(number: u32) -> Option<SessionPhase> {
    u8::try_from(number)
        .ok()
        .and_then(|n| SessionPhaseWire(n).to_known())
}

/// The wait after the `n`th fault ending in a row: 15, 30, 60, 120, 120, ….
fn backoff_s(n: u32) -> u64 {
    let doublings = n.saturating_sub(1).min(4);
    (BACKOFF_FIRST_S << doublings).min(BACKOFF_CAP_S)
}

/// Where a script's clip starts, by shape.
const fn play_ms(shape: Shape) -> u64 {
    match shape {
        Shape::Opening => OPEN_PLAY_MS,
        Shape::Replacement => SEAM_PLAY_MS,
    }
}

/// When an `active` that began at `since` has lasted long enough to reset the
/// ladder.
fn ladder_reset_at(since: SyncTime) -> SyncTime {
    plus_ms(since, LADDER_RESET_S.saturating_mul(1_000))
}

/// The body as a script, or `None` when it is not one.
fn decoded(body: &[u8]) -> Option<MotionScript> {
    std::str::from_utf8(body)
        .ok()
        .and_then(|text| MotionScript::decode(text).ok())
}

/// Whether `script` plays nothing and names no pose but the stow.
fn bare_stow(script: &MotionScript) -> bool {
    let mut posed = false;
    for step in script.steps() {
        match &step.action {
            Action::Play(_) => return false,
            Action::Base(Base::Pose { name, .. }) => {
                if name != STOW_POSE {
                    return false;
                }
                posed = true;
            }
            Action::Base(Base::Keep) => {}
        }
    }
    posed
}

fn nanos_of_ms(ms: u64) -> i64 {
    i64::try_from(ms)
        .unwrap_or(i64::MAX)
        .saturating_mul(1_000_000)
}

fn plus_ms(t: SyncTime, ms: u64) -> SyncTime {
    SyncTime::from_nanos(t.as_nanos().saturating_add(nanos_of_ms(ms)))
}

fn minus_ms(t: SyncTime, ms: u64) -> SyncTime {
    SyncTime::from_nanos(t.as_nanos().saturating_sub(nanos_of_ms(ms)))
}

/// Milliseconds since the epoch, what a sequence number is drawn from.
///
/// An instant before the epoch — a clock never set — reads as zero, the
/// same convention as `motion_proto::unix_millis`: `SeqSource` still
/// numbers strictly upward from its last issue.
fn unix_ms(t: SyncTime) -> u64 {
    u64::try_from(t.as_nanos() / 1_000_000).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
    use motion_proto::PlayWindow;
    use reachy_edge::{EdgeConfig, HostEdge};

    const POD: &str = "fixture-reachy";

    /// A round epoch the fixture's instants are counted from.
    const T0_MS: u64 = 1_800_000_000_000;

    const SEED: u64 = 0x5EED_1DEA_0000_0042;

    fn t(ms: u64) -> SyncTime {
        SyncTime::from_nanos(i64::try_from((T0_MS + ms) * 1_000_000).expect("in range"))
    }

    fn ms_of(at: SyncTime) -> u64 {
        u64::try_from(at.as_nanos() / 1_000_000).expect("after the epoch") - T0_MS
    }

    fn entry(motion_id: u16, duration_ms: u64, blend_out_ms: u64) -> MotionEntry {
        MotionEntry {
            motion_id,
            window: PlayWindow {
                duration_ms,
                blend_out_ms,
            },
        }
    }

    fn table() -> MotionTable {
        MotionTable::of([
            ("a/one".to_owned(), entry(10, 2000, 200)),
            ("a/two".to_owned(), entry(11, 3000, 200)),
            ("a/three".to_owned(), entry(12, 1500, 400)),
            ("probe/antenna-x".to_owned(), entry(20, 500, 200)),
            ("bench/nod".to_owned(), entry(21, 800, 200)),
        ])
    }

    fn window(name: &str) -> PlayWindow {
        table().resolve(name).expect("a fixture motion").window
    }

    fn playlist() -> Playlist {
        Playlist::parse(r#"{"playlist":["a/one","a/two","a/three"]}"#, &table())
            .expect("the fixture playlist")
    }

    /// A surface that keeps its lines.
    #[derive(Debug, Default)]
    struct Recorded {
        lines: Vec<String>,
    }

    impl Surface for Recorded {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }

        fn alert(&mut self, _alert: &reachy_edge::Alert) {}
    }

    impl Recorded {
        fn of(&self, word: &str) -> Vec<Value> {
            self.lines
                .iter()
                .map(|line| serde_json::from_str::<Value>(line).expect("one JSON object"))
                .filter(|line| line["kind"] == word)
                .collect()
        }

        fn count(&self, word: &str) -> usize {
            self.of(word).len()
        }
    }

    fn row(kind: ReportKindWire, a: u32, b: u32, t_ms: u64) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(t(t_ms));
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry
    }

    fn update(rows: Vec<TimelineEntryWire>) -> Update {
        Update {
            restarted: false,
            lost: 0,
            rows,
        }
    }

    fn phase(entered: SessionPhaseWire, left: SessionPhaseWire, t_ms: u64) -> TimelineEntryWire {
        row(
            ReportKindWire::PHASE_CHANGED,
            u32::from(entered.0),
            u32::from(left.0),
            t_ms,
        )
    }

    fn ended(script_id: u32, t_ms: u64) -> TimelineEntryWire {
        row(ReportKindWire::SESSION_ENDED, script_id, 0, t_ms)
    }

    fn refused_row(script_id: u32, reason: u32, t_ms: u64) -> TimelineEntryWire {
        row(ReportKindWire::SCRIPT_REFUSED, script_id, reason, t_ms)
    }

    /// A speech script for [`POD`], encoded.
    fn body(steps: Vec<Step>, timeout_ms: u64) -> Vec<u8> {
        numbered(1, steps, timeout_ms)
    }

    /// A speech script for [`POD`] numbered `seq`, encoded.
    fn numbered(seq: u64, steps: Vec<Step>, timeout_ms: u64) -> Vec<u8> {
        MotionScript::new(POD, seq, steps, timeout_ms)
            .expect("a lawful speech script")
            .encode()
            .into_bytes()
    }

    fn hold() -> Vec<u8> {
        body(vec![Step::new(0, "listen")], 30_000)
    }

    fn closing(stow_ms: u64) -> Vec<u8> {
        body(
            vec![Step::new(0, "listen"), Step::new(stow_ms, STOW_POSE)],
            stow_ms + 3_000,
        )
    }

    fn bare_stow_body() -> Vec<u8> {
        body(vec![Step::new(0, STOW_POSE)], 3_000)
    }

    fn motion(script: &MotionScript) -> &str {
        &script.steps()[1]
            .action
            .play()
            .expect("a loop script's second step plays")
            .name
    }

    /// The script's steps are the loop's shape with its clip at `play`.
    fn assert_shape(script: &MotionScript, play: u64) {
        assert_eq!(script.pod(), POD);
        let name = motion(script);
        let w = window(name);
        let stow = play + w.duration_ms + w.blend_out_ms + STOW_MARGIN_MS;
        assert_eq!(
            script.steps(),
            &[
                Step::new(0, NEUTRAL_POSE),
                Step::play(play, Play::new(name)),
                Step::new(stow, STOW_POSE),
            ],
        );
        assert!(matches!(
            &script.steps()[2].action,
            Action::Base(Base::Pose { name, .. }) if name == STOW_POSE
        ));
        assert_eq!(script.timeout_ms(), stow + STOW_DURATION_MS);
    }

    /// The loop, what it narrated, and the ids an edge would have issued.
    struct Rig {
        idle: Idle,
        surface: Recorded,
        next_id: u32,
    }

    impl Rig {
        fn new(seed: u64) -> Self {
            Self {
                idle: Idle::new(playlist(), POD, seed),
                surface: Recorded::default(),
                next_id: 1,
            }
        }

        fn rows(&mut self, rows: Vec<TimelineEntryWire>) {
            self.idle.story(&update(rows), &mut self.surface);
        }

        fn engage(&mut self, at: u64) {
            self.rows(vec![
                phase(
                    SessionPhaseWire::ENGAGING,
                    SessionPhaseWire::RESTING,
                    at + 100,
                ),
                phase(
                    SessionPhaseWire::ACTIVE,
                    SessionPhaseWire::ENGAGING,
                    at + 1_000,
                ),
            ]);
        }

        /// A poll, checked for a `due` strictly after it.
        fn poll(&mut self, ms: u64) -> Poll {
            let poll = self.idle.poll(t(ms), &mut self.surface);
            if let Some(due) = poll.due {
                assert!(due > t(ms), "due {} at {ms}", ms_of(due));
            }
            poll
        }

        fn quiet(&mut self, ms: u64) -> Poll {
            let poll = self.poll(ms);
            assert!(poll.send.is_none(), "sent at {ms}: {:?}", poll.send);
            poll
        }

        /// A poll that must send, its script not yet answered.
        fn offer(&mut self, ms: u64) -> MotionScript {
            self.poll(ms).send.expect("a script to send")
        }

        fn sent(&mut self, ms: u64) -> u32 {
            let id = self.next_id;
            self.next_id += 1;
            self.idle.sent(id, t(ms), &mut self.surface);
            id
        }

        /// A poll that sends, accepted by the edge at the same instant.
        fn send(&mut self, ms: u64) -> (u32, MotionScript) {
            let script = self.offer(ms);
            (self.sent(ms), script)
        }

        fn tick(&mut self, ms: u64) {
            if self.poll(ms).send.is_some() {
                self.sent(ms);
            }
        }

        fn speech(&mut self, body: &[u8], ms: u64) -> Verdict {
            self.idle
                .speech(body, Origin::Local, t(ms), &mut self.surface)
        }

        fn accept(&mut self, body: &[u8], ms: u64) {
            self.idle.speech_accepted(body, t(ms), &mut self.surface);
        }

        fn standing_due(&self) -> u64 {
            ms_of(self.idle.standing.expect("a standing script").due)
        }

        fn last_backoff(&self) -> u64 {
            self.surface
                .of(IDLE_BACKOFF)
                .last()
                .expect("a backoff line")["seconds"]
                .as_u64()
                .expect("a number")
        }
    }

    /// Resting after a fresh commission.
    fn resting_loop() -> Rig {
        let mut rig = Rig::new(SEED);
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            0,
        )]);
        rig
    }

    /// Its opening sent at 0, then engaged, and `active` from 1000.
    fn active_loop() -> Rig {
        let mut rig = resting_loop();
        rig.send(0);
        rig.engage(0);
        rig
    }

    /// A session ending on a fault at `ms`, with no `session_ended`. Returns
    /// when `resting` was entered.
    fn fault_end(rig: &mut Rig, ms: u64) -> u64 {
        rig.rows(vec![
            phase(SessionPhaseWire::WINDING_DOWN, SessionPhaseWire::ACTIVE, ms),
            phase(
                SessionPhaseWire::STOPPING,
                SessionPhaseWire::WINDING_DOWN,
                ms + 1,
            ),
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                ms + 2,
            ),
        ]);
        ms + 2
    }

    /// A session ending cleanly at `ms`. Returns when `resting` was entered.
    fn clean_end(rig: &mut Rig, ms: u64) -> u64 {
        rig.rows(vec![
            phase(SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE, ms),
            ended(1, ms + 1),
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                ms + 2,
            ),
        ]);
        ms + 2
    }

    /// From `active` since `active_at`: a fault ending `active_for` later, the
    /// backoff waited out, and the next opening engaged. Returns the backoff's
    /// seconds and when the next `active` began.
    fn fault_cycle(rig: &mut Rig, active_at: u64, active_for: u64) -> (u64, u64) {
        let rest = fault_end(rig, active_at + active_for);
        let wait = rig.last_backoff();
        let open = rest + wait * 1_000;
        rig.quiet(open - 1);
        rig.send(open);
        rig.engage(open);
        (wait, open + 1_000)
    }

    /// The loop has something due, or one more phase row gets it sending or
    /// waiting on something.
    fn assert_moves(rig: &mut Rig, ms: u64, next: TimelineEntryWire) {
        let poll = rig.poll(ms);
        if poll.send.is_some() || poll.due.is_some() {
            return;
        }
        rig.rows(vec![next]);
        let poll = rig.poll(ms + 10);
        assert!(
            poll.send.is_some() || poll.due.is_some(),
            "a loop that nothing moves"
        );
    }

    #[test]
    fn a_good_playlist_keeps_file_order() {
        let playlist = Playlist::parse(r#"{"playlist":["a/three","a/one"]}"#, &table())
            .expect("a good playlist");
        let names: Vec<&str> = playlist.entries().map(|(name, _)| name).collect();
        assert_eq!(names, ["a/three", "a/one"]);
        assert_eq!(playlist.len(), 2);
        assert!(!playlist.is_empty());
        assert_eq!(
            playlist.entries().next().map(|(_, entry)| entry),
            Some(entry(12, 1500, 400))
        );
    }

    #[test]
    fn a_probe_or_bench_motion_is_excluded() {
        for name in ["probe/antenna-x", "bench/nod"] {
            let text = format!(r#"{{"playlist":["a/one","{name}","a/two"]}}"#);
            assert_eq!(
                Playlist::parse(&text, &table()),
                Err(IdleError::Excluded {
                    name: name.to_owned()
                })
            );
        }
    }

    #[test]
    fn a_repeated_name_is_a_duplicate() {
        assert_eq!(
            Playlist::parse(r#"{"playlist":["a/one","a/two","a/one"]}"#, &table()),
            Err(IdleError::Duplicate {
                name: "a/one".to_owned()
            })
        );
    }

    #[test]
    fn a_name_the_table_does_not_hold_is_unknown() {
        assert_eq!(
            Playlist::parse(r#"{"playlist":["a/one","a/four"]}"#, &table()),
            Err(IdleError::UnknownMotion {
                name: "a/four".to_owned()
            })
        );
    }

    #[test]
    fn fewer_than_two_motions_is_too_few() {
        assert_eq!(
            Playlist::parse(r#"{"playlist":[]}"#, &table()),
            Err(IdleError::TooFew { count: 0 })
        );
        assert_eq!(
            Playlist::parse(r#"{"playlist":["a/one"]}"#, &table()),
            Err(IdleError::TooFew { count: 1 })
        );
    }

    #[test]
    fn a_file_of_another_shape_is_malformed() {
        for text in [
            r#"["a/one","a/two"]"#,
            r#""a/one""#,
            r#"{"playlist":["a/one","a/two"],"extra":1}"#,
            r#"{"playlists":["a/one","a/two"]}"#,
        ] {
            assert!(
                matches!(
                    Playlist::parse(text, &table()),
                    Err(IdleError::Malformed { .. })
                ),
                "{text}"
            );
        }
    }

    #[test]
    fn load_reads_the_file_and_names_a_missing_one() {
        let dir = reachy_scratch::scratch_dir("reachy-host-idle-load");
        let missing = dir.join("absent.json");
        let error = Idle::load(&missing, &table()).expect_err("a path that is not there");
        assert!(
            matches!(&error, IdleError::Read { path, .. } if *path == missing),
            "{error:?}"
        );
        assert!(error.to_string().contains(&missing.display().to_string()));

        let good = dir.join("idle.json");
        let text = r#"{"playlist":["a/one","a/two","a/three"]}"#;
        std::fs::write(&good, text).expect("a scratch file");
        assert_eq!(Idle::load(&good, &table()), Ok(playlist()));
    }

    #[test]
    fn a_fresh_boot_opens_at_once_with_the_opening_shape() {
        let mut rig = resting_loop();
        let script = rig.offer(0);
        assert_shape(&script, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 0);
    }

    #[test]
    fn a_replacement_at_the_standing_due_has_the_seam_shape() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        let (_, script) = rig.send(due);
        assert_shape(&script, SEAM_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_REPLACED), 1);
    }

    /// The two poses the loop names, the stow at the deployed library's own
    /// budget: the loop sizes every timeout on it, and the edge refuses a
    /// timeline that leaves no room for the stow.
    const POSES: &str = r#"{
  "motions": [],
  "poses": [
    {"pose_id": 0, "name": "neutral", "duration_ms": 800},
    {"pose_id": 4, "name": "stow", "duration_ms": 2000}
  ]
}"#;

    #[test]
    fn every_shape_over_the_playlist_compiles_at_the_edge() {
        let (_, poses) = reachy_edge::parse(POSES).expect("a library holding the stow");
        assert_eq!(u64::from(poses.stow().duration_ms), STOW_DURATION_MS);
        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), table(), poses);
        let mut idle = Idle::new(playlist(), POD, SEED);
        let mut surface = Recorded::default();
        let mut at = 0;
        for index in 0..idle.playlist.len() {
            for shape in [Shape::Opening, Shape::Replacement] {
                let script = idle
                    .script_for(index, shape, t(at))
                    .expect("a fixed shape over a checked playlist");
                let accepted = host.offer(
                    script.encode().as_bytes(),
                    Origin::Local,
                    t(at),
                    &mut surface,
                );
                assert!(accepted.is_some(), "{index} {shape:?}: {:?}", surface.lines);
                at += 1;
            }
        }
    }

    #[test]
    fn the_due_is_the_clip_end_less_the_lead() {
        let mut rig = resting_loop();
        let (_, opening) = rig.send(0);
        rig.engage(0);
        let due = OPEN_PLAY_MS + window(motion(&opening)).duration_ms - SEAM_LEAD_MS;
        assert_eq!(rig.poll(1_000).due, Some(t(due)));

        rig.quiet(due - 1);
        let (_, replacement) = rig.send(due);
        let next = due + SEAM_PLAY_MS + window(motion(&replacement)).duration_ms - SEAM_LEAD_MS;
        assert_eq!(rig.poll(due).due, Some(t(next)));
        rig.quiet(next - 1);
        assert!(rig.poll(next).send.is_some());
    }

    #[test]
    fn nothing_is_sent_while_the_session_is_between_turns() {
        let mut rig = resting_loop();
        rig.send(0);
        rig.rows(vec![phase(
            SessionPhaseWire::ENGAGING,
            SessionPhaseWire::RESTING,
            100,
        )]);
        for ms in [200, 10_000, 60_000] {
            rig.quiet(ms);
        }

        let mut rig = active_loop();
        rig.rows(vec![phase(
            SessionPhaseWire::WINDING_DOWN,
            SessionPhaseWire::ACTIVE,
            1_500,
        )]);
        for ms in [2_000, 10_000, 60_000] {
            rig.quiet(ms);
        }
        rig.rows(vec![phase(
            SessionPhaseWire::STOPPING,
            SessionPhaseWire::WINDING_DOWN,
            60_001,
        )]);
        for ms in [60_002, 70_000, 120_000] {
            rig.quiet(ms);
        }
    }

    #[test]
    fn nothing_is_sent_before_any_phase_row() {
        let mut rig = Rig::new(SEED);
        assert_eq!(rig.quiet(0), Poll::default());
        assert_eq!(rig.quiet(1_000), Poll::default());
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 0);
        let probe = rig.offer(600_000);
        assert_shape(&probe, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 1);
        assert_eq!(rig.quiet(700_000), Poll::default());
        assert_eq!(rig.quiet(800_000), Poll::default());
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 1);
    }

    #[test]
    fn a_story_before_the_grace_says_nothing() {
        let mut rig = Rig::new(SEED);
        rig.quiet(0);
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            5_000,
        )]);
        rig.send(5_000);
        rig.quiet(60_000);
        assert_eq!(rig.surface.count(IDLE_OPENED), 1);
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 0);
    }

    /// Polled at 0 and just short of the grace, then offered the probe at the
    /// grace and the edge's acceptance of it as id 1.
    fn probed_loop() -> Rig {
        let mut rig = Rig::new(SEED);
        rig.quiet(0);
        rig.quiet(9_999);
        let probe = rig.offer(10_000);
        assert_shape(&probe, OPEN_PLAY_MS);
        assert_eq!(rig.sent(10_000), 1);
        rig
    }

    #[test]
    fn a_loop_that_hears_no_story_offers_one_opening_at_the_grace() {
        let mut rig = probed_loop();
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 1);
        assert_eq!(rig.surface.count(IDLE_OPENED), 1);
        rig.quiet(10_001);
        rig.quiet(60_000);
        rig.quiet(600_000);
        assert_eq!(rig.surface.count(IDLE_NO_STORY), 1);
    }

    #[test]
    fn a_late_host_s_replayed_ring_clears_the_probe_and_the_loop_reopens_after_the_clip() {
        let mut rig = probed_loop();
        rig.rows(vec![
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STARTING, 3_000),
            row(ReportKindWire::SCRIPT_ACCEPTED, 1, 2, 10_001),
            phase(
                SessionPhaseWire::ENGAGING,
                SessionPhaseWire::RESTING,
                10_001,
            ),
            phase(SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING, 10_900),
        ]);
        assert!(rig.idle.standing.is_none());
        rig.quiet(11_000);
        rig.quiet(20_000);
        let rest = clean_end(&mut rig, 25_000);
        let (_, s) = rig.send(rest);
        assert_shape(&s, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_OPENED), 2);
    }

    #[test]
    fn a_late_host_whose_first_datagram_is_only_the_boot_row_opens_again_from_resting() {
        let mut rig = probed_loop();
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            3_000,
        )]);
        let (_, s) = rig.send(10_100);
        assert_shape(&s, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_OPENED), 2);
        rig.engage(10_100);
        assert!(rig.idle.standing.is_some());
        let due = rig.standing_due();
        rig.quiet(due - 1);
        let (_, r) = rig.send(due);
        assert_shape(&r, SEAM_PLAY_MS);
    }

    #[test]
    fn a_probe_held_at_starting_is_cleared_by_the_boot_row_and_the_loop_reopens_after_the_clip() {
        let mut rig = probed_loop();
        rig.rows(vec![row(
            ReportKindWire::SCRIPT_HELD,
            1,
            u32::from(SessionPhaseWire::STARTING.0),
            10_001,
        )]);
        assert!(rig.idle.standing.is_some());
        rig.quiet(10_500);
        rig.rows(vec![
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STARTING,
                12_000,
            ),
            row(ReportKindWire::SCRIPT_ACCEPTED, 1, 2, 12_000),
            phase(
                SessionPhaseWire::ENGAGING,
                SessionPhaseWire::RESTING,
                12_000,
            ),
            phase(SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING, 12_900),
        ]);
        assert!(rig.idle.standing.is_none());
        rig.quiet(13_000);
        rig.quiet(20_000);
        let rest = clean_end(&mut rig, 25_000);
        let (_, s) = rig.send(rest);
        assert_shape(&s, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_OPENED), 2);
    }

    #[test]
    fn a_probe_into_a_parked_machine_parks_the_loop() {
        let mut rig = probed_loop();
        rig.rows(vec![
            row(ReportKindWire::COMMISSION_FAILED, 0, 0, 2_999),
            phase(SessionPhaseWire::PARKED, SessionPhaseWire::STARTING, 3_000),
            refused_row(1, u32::from(RefusalReasonWire::PARKED.0), 10_001),
        ]);
        assert_eq!(rig.surface.count(IDLE_PARKED), 1);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 1);
        assert_eq!(rig.surface.of(IDLE_REFUSED)[0]["reason"], "session");
        for n in 0..50 {
            assert_eq!(rig.quiet(10_002 + n * 1_000), Poll::default());
        }
    }

    #[test]
    fn an_unsent_probe_halts_until_the_boot_rows() {
        let mut rig = Rig::new(SEED);
        rig.quiet(0);
        let probe = rig.offer(10_000);
        assert_shape(&probe, OPEN_PLAY_MS);
        rig.idle.unsent(1, t(10_000), &mut rig.surface);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 1);
        assert_eq!(rig.surface.of(IDLE_REFUSED)[0]["reason"], "unsent");
        rig.quiet(20_000);
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            30_000,
        )]);
        let (_, s) = rig.send(30_000);
        assert_shape(&s, OPEN_PLAY_MS);
    }

    #[test]
    fn nothing_is_sent_after_a_park() {
        let mut rig = active_loop();
        rig.rows(vec![phase(
            SessionPhaseWire::PARKED,
            SessionPhaseWire::ACTIVE,
            1_500,
        )]);
        for ms in [2_000, 100_000, 10_000_000] {
            assert_eq!(rig.quiet(ms), Poll::default());
        }
    }

    #[test]
    fn nothing_is_sent_while_halted() {
        let mut rig = resting_loop();
        rig.offer(0);
        rig.idle.refused(t(0), &mut rig.surface);
        for ms in [1, 1_000, 100_000] {
            rig.quiet(ms);
        }
    }

    #[test]
    fn nothing_is_sent_at_resting_before_the_resume() {
        let mut rig = active_loop();
        let rest = fault_end(&mut rig, 3_000);
        rig.quiet(rest);
        rig.quiet(rest + 14_999);
    }

    #[test]
    fn an_opening_sent_at_resting_is_not_sent_again() {
        let mut rig = resting_loop();
        rig.send(0);
        for n in 0..10 {
            rig.quiet(n * 1_000);
        }
    }

    #[test]
    fn nothing_is_sent_while_speech_holds_the_head_before_its_resume() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        rig.accept(&closing(8_000), 1_500);
        rig.quiet(due);
        rig.quiet(1_500 + 8_000 - SEAM_LEAD_MS - 1);
    }

    #[test]
    fn classifying_a_body_changes_nothing() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        let lines = rig.surface.lines.len();
        assert_eq!(rig.speech(&hold(), 1_500), Verdict::Offer);
        assert_eq!(rig.speech(&closing(8_000), 1_600), Verdict::Offer);
        assert_eq!(rig.idle.owner, Owner::Loop);
        assert_eq!(rig.idle.resume_at, None);
        assert_eq!(rig.surface.lines.len(), lines);
        let (_, script) = rig.send(due);
        assert_shape(&script, SEAM_PLAY_MS);
        assert_eq!(rig.idle.owner, Owner::Loop);
    }

    #[test]
    fn only_an_accepted_hold_takes_the_head() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        rig.accept(&closing(8_000), 1_500);
        assert!(rig.idle.resume_at.is_some());
        rig.accept(&hold(), 1_600);
        assert_eq!(rig.surface.count(IDLE_SUSPENDED), 1);
        assert_eq!(rig.idle.owner, Owner::Speech);
        assert_eq!(rig.idle.resume_at, None);
        assert_eq!(rig.idle.standing, None);
        rig.quiet(due);
    }

    #[test]
    fn a_refused_speech_body_leaves_the_replacement_standing() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        assert_eq!(rig.speech(&hold(), 1_500), Verdict::Offer);
        let (_, script) = rig.send(due);
        assert_shape(&script, SEAM_PLAY_MS);
    }

    #[test]
    fn a_refused_speech_body_leaves_the_backoff_standing() {
        let mut rig = active_loop();
        let rest = fault_end(&mut rig, 6_000);
        assert_eq!(rig.speech(&hold(), rest + 1_000), Verdict::Offer);
        rig.quiet(rest + 14_999);
        let (_, script) = rig.send(rest + 15_000);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_closing_script_resumes_the_loop_at_its_stow_less_the_lead() {
        let mut rig = active_loop();
        rig.accept(&closing(8_000), 2_000);
        let resume = 2_000 + 8_000 - SEAM_LEAD_MS;
        assert_eq!(rig.quiet(resume - 1).due, Some(t(resume)));

        // The same instant, re-emitted 5 s later with its offset recomputed.
        rig.accept(&closing(3_000), 7_000);
        assert_eq!(rig.idle.resume_at, Some(t(resume)));
        assert_eq!(rig.surface.count(IDLE_SUSPENDED), 1);

        let (_, script) = rig.send(resume);
        assert_shape(&script, SEAM_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_RESUMED), 1);
        assert_eq!(rig.surface.count(IDLE_OPENED), 1);
        assert_eq!(rig.idle.owner, Owner::Loop);

        let next = rig.standing_due();
        assert_eq!(
            next,
            resume + SEAM_PLAY_MS + window(motion(&script)).duration_ms - SEAM_LEAD_MS
        );
        rig.quiet(next - 1);
        rig.send(next);
        assert_eq!(rig.surface.count(IDLE_REPLACED), 1);
    }

    #[test]
    fn a_hold_after_a_closing_clears_the_resume() {
        let mut rig = active_loop();
        rig.accept(&closing(8_000), 2_000);
        rig.accept(&hold(), 3_000);
        assert_eq!(rig.idle.resume_at, None);
        assert_eq!(rig.quiet(20_000).due, None);
    }

    #[test]
    fn a_bare_stow_is_dropped_while_the_loop_holds_the_head() {
        let mut rig = active_loop();
        assert_eq!(rig.speech(&bare_stow_body(), 1_500), Verdict::Drop);
        let dropped = rig.surface.of(IDLE_DROPPED_STOW);
        assert_eq!(dropped.len(), 1);
        assert_eq!(dropped[0]["origin"], "local");
    }

    #[test]
    fn a_bare_stow_is_offered_while_speech_holds_the_head() {
        let mut rig = active_loop();
        rig.accept(&hold(), 1_500);
        let lines = rig.surface.lines.len();
        assert_eq!(rig.speech(&bare_stow_body(), 1_600), Verdict::Offer);
        assert_eq!(rig.surface.lines.len(), lines);
    }

    #[test]
    fn an_accepted_bare_stow_changes_nothing() {
        for speech_first in [false, true] {
            let mut rig = active_loop();
            if speech_first {
                rig.accept(&closing(8_000), 1_500);
            }
            let before = (
                rig.idle.owner,
                rig.idle.standing,
                rig.idle.resume_at,
                rig.surface.lines.len(),
            );
            rig.accept(&bare_stow_body(), 1_600);
            let after = (
                rig.idle.owner,
                rig.idle.standing,
                rig.idle.resume_at,
                rig.surface.lines.len(),
            );
            assert_eq!(before, after);
        }
    }

    #[test]
    fn resting_hands_the_head_back_to_the_loop() {
        let mut rig = active_loop();
        rig.accept(&hold(), 1_500);
        let rest = clean_end(&mut rig, 30_000);
        assert_eq!(rig.idle.owner, Owner::Loop);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_speech_body_held_across_resting_keeps_the_head() {
        let mut rig = active_loop();
        rig.rows(vec![phase(
            SessionPhaseWire::STOPPING,
            SessionPhaseWire::ACTIVE,
            30_000,
        )]);
        rig.accept(&hold(), 30_500);
        rig.rows(vec![
            ended(1, 31_000),
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                31_001,
            ),
        ]);
        assert_eq!(rig.idle.owner, Owner::Speech);
        rig.quiet(31_001);
        rig.quiet(40_000);
        rig.engage(31_000);
        rig.quiet(45_000);
        let rest = clean_end(&mut rig, 60_000);
        assert_eq!(rig.idle.owner, Owner::Loop);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_closing_held_across_resting_resumes_at_its_stow() {
        let mut rig = active_loop();
        rig.rows(vec![phase(
            SessionPhaseWire::STOPPING,
            SessionPhaseWire::ACTIVE,
            30_000,
        )]);
        rig.accept(&closing(8_000), 30_500);
        rig.rows(vec![
            ended(1, 31_000),
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                31_001,
            ),
        ]);
        rig.engage(31_000);
        let resume = 30_500 + 8_000 - SEAM_LEAD_MS;
        rig.quiet(resume - 1);
        let (_, script) = rig.send(resume);
        assert_shape(&script, SEAM_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_RESUMED), 1);
    }

    #[test]
    fn a_speech_body_held_at_engaging_is_done_with_at_its_own_resting() {
        let mut rig = resting_loop();
        rig.send(0);
        rig.rows(vec![phase(
            SessionPhaseWire::ENGAGING,
            SessionPhaseWire::RESTING,
            100,
        )]);
        rig.accept(&hold(), 200);
        rig.rows(vec![phase(
            SessionPhaseWire::ACTIVE,
            SessionPhaseWire::ENGAGING,
            1_000,
        )]);
        let rest = clean_end(&mut rig, 30_000);
        assert_eq!(rig.idle.owner, Owner::Loop);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_resume_on_the_accepting_poll_is_numbered_above_the_speech_body() {
        let mut rig = active_loop();
        let seq = unix_ms(t(2_000));
        rig.accept(
            &numbered(
                seq,
                vec![Step::new(0, "listen"), Step::new(40, STOW_POSE)],
                3_040,
            ),
            2_000,
        );
        let (_, script) = rig.send(2_000);
        assert!(
            script.seq() > seq,
            "the loop's {} is not above speech's {seq}",
            script.seq()
        );
        assert_eq!(rig.surface.count(IDLE_RESUMED), 1);
    }

    #[test]
    fn an_ending_with_session_ended_is_clean() {
        let mut rig = active_loop();
        let rest = clean_end(&mut rig, 6_000);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 0);
        rig.send(rest);
    }

    #[test]
    fn an_ending_without_session_ended_is_a_fault() {
        let mut rig = active_loop();
        let rest = fault_end(&mut rig, 6_000);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 1);
        assert_eq!(rig.last_backoff(), 15);
        rig.quiet(rest + 14_999);
        rig.send(rest + 15_000);
    }

    #[test]
    fn a_degraded_session_that_ended_is_clean() {
        let mut rig = active_loop();
        rig.rows(vec![
            row(ReportKindWire::FAULT_RECORDED, 7, 17, 3_000),
            row(ReportKindWire::RESPONSE_TAKEN, 3, 7, 3_001),
            phase(SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE, 6_000),
            ended(1, 6_001),
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING, 6_002),
        ]);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 0);
        assert_eq!(rig.idle.consecutive_faults, 0);
        rig.send(6_002);
    }

    #[test]
    fn the_first_resting_after_starting_is_no_ending() {
        let mut rig = active_loop();
        fault_end(&mut rig, 6_000);
        assert_eq!(rig.idle.consecutive_faults, 1);
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            7_000,
        )]);
        assert_eq!(rig.idle.consecutive_faults, 1);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 1);
        rig.send(7_000);
    }

    #[test]
    fn repeated_faults_climb_the_ladder_to_its_cap() {
        let mut rig = active_loop();
        let mut active_at = 1_000;
        let mut seen = Vec::new();
        for _ in 0..5 {
            let (wait, next) = fault_cycle(&mut rig, active_at, 5_000);
            seen.push(wait);
            active_at = next;
        }
        assert_eq!(seen, [15, 30, 60, 120, 120]);
    }

    #[test]
    fn an_active_under_a_minute_does_not_reset_the_ladder() {
        let mut rig = active_loop();
        let (_, a) = fault_cycle(&mut rig, 1_000, 5_000);
        let (_, a) = fault_cycle(&mut rig, a, 5_000);
        rig.tick(a + 59_999);
        let (wait, _) = fault_cycle(&mut rig, a, 59_999);
        assert_eq!(wait, 60);
    }

    #[test]
    fn a_minute_of_active_resets_the_ladder() {
        let mut rig = active_loop();
        let (_, a) = fault_cycle(&mut rig, 1_000, 5_000);
        let (_, a) = fault_cycle(&mut rig, a, 5_000);
        rig.tick(a + 60_000);
        assert_eq!(rig.idle.consecutive_faults, 0);
        let (wait, _) = fault_cycle(&mut rig, a, 60_000);
        assert_eq!(wait, 15);
    }

    #[test]
    fn the_ladder_reset_is_a_due() {
        let mut rig = active_loop();
        let (_, a) = fault_cycle(&mut rig, 1_000, 5_000);
        rig.accept(&hold(), a + 10);
        assert_eq!(rig.quiet(a + 20).due, Some(t(a + 60_000)));
    }

    #[test]
    fn a_lost_resting_row_is_answered_by_the_replacement() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        let (_, script) = rig.send(due);
        assert_shape(&script, SEAM_PLAY_MS);
        rig.engage(due);
        let poll = rig.quiet(due + 1_000);
        assert!(poll.due.is_some());
        assert_moves(
            &mut rig,
            due + 1_000,
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                due + 1_001,
            ),
        );
    }

    #[test]
    fn a_lost_session_ended_costs_one_backoff_step() {
        let mut rig = active_loop();
        rig.rows(vec![
            phase(SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE, 6_000),
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING, 6_001),
        ]);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 1);
        assert_moves(
            &mut rig,
            6_001,
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING, 6_002),
        );
        rig.quiet(21_000);
        let (_, script) = rig.send(21_001);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_lost_parked_row_is_answered_by_the_refusal_once() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        let (id, _) = rig.send(due);
        rig.rows(vec![refused_row(id, 1, due + 5)]);
        assert!(rig.idle.halted);
        for n in 0..20 {
            rig.quiet(due + 10 + n * 30_000);
        }
        let refusals = rig.surface.of(IDLE_REFUSED);
        assert_eq!(refusals.len(), 1);
        assert_eq!(refusals[0]["script_id"], id);
        assert_eq!(refusals[0]["reason"], "session");
        assert_eq!(refusals[0]["refusal"], 1);
        assert_moves(
            &mut rig,
            due + 700_000,
            phase(
                SessionPhaseWire::RESTING,
                SessionPhaseWire::STOPPING,
                due + 700_001,
            ),
        );
    }

    fn assert_no_repeats(seed: u64, seams: usize) {
        let mut rig = Rig::new(seed);
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            0,
        )]);
        let (_, first) = rig.send(0);
        rig.engage(0);
        let mut names = vec![motion(&first).to_owned()];
        for _ in 0..seams {
            let at = rig.standing_due();
            let (_, script) = rig.send(at);
            names.push(motion(&script).to_owned());
        }
        for pair in names.windows(2) {
            assert_ne!(pair[0], pair[1], "{names:?}");
        }
    }

    #[test]
    fn no_motion_plays_twice_in_a_row() {
        assert_no_repeats(0xA5A5_1234_DEAD_BEEF, 200);
        assert_no_repeats(0, 20);
    }

    #[test]
    fn a_restart_after_a_park_opens_a_fresh_machine_run() {
        let mut rig = resting_loop();
        let (_, old) = rig.send(10_000);
        rig.engage(10_000);
        fault_end(&mut rig, 12_000);
        rig.rows(vec![phase(
            SessionPhaseWire::PARKED,
            SessionPhaseWire::RESTING,
            12_100,
        )]);
        assert!(rig.idle.parked);
        rig.quiet(100_000);

        rig.idle.story(
            &Update {
                restarted: true,
                lost: 0,
                rows: vec![phase(
                    SessionPhaseWire::RESTING,
                    SessionPhaseWire::STARTING,
                    4_000,
                )],
            },
            &mut rig.surface,
        );
        assert!(!rig.idle.parked);
        assert_eq!(rig.idle.consecutive_faults, 0);
        assert_eq!(rig.idle.owner, Owner::Loop);
        // The clock read earlier than the first opening: the sequence still
        // climbs.
        let (_, new) = rig.send(5_000);
        assert_shape(&new, OPEN_PLAY_MS);
        assert!(new.seq() > old.seq(), "{} after {}", new.seq(), old.seq());
    }

    #[test]
    fn a_restart_takes_the_head_back_from_speech() {
        let mut rig = active_loop();
        rig.accept(&hold(), 1_500);
        rig.idle.story(
            &Update {
                restarted: true,
                lost: 0,
                rows: vec![phase(
                    SessionPhaseWire::RESTING,
                    SessionPhaseWire::STARTING,
                    3_000,
                )],
            },
            &mut rig.surface,
        );
        assert_eq!(rig.idle.owner, Owner::Loop);
        let (_, script) = rig.send(3_000);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn an_edge_refusal_halts_until_the_next_phase_row() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        rig.offer(due);
        rig.idle.refused(t(due), &mut rig.surface);
        for n in 0..50 {
            rig.quiet(due + 1 + n * 12_000);
        }
        let refusals = rig.surface.of(IDLE_REFUSED);
        assert_eq!(refusals.len(), 1);
        assert_eq!(refusals[0]["script_id"], Value::Null);
        assert_eq!(refusals[0]["reason"], "edge");

        let rest = clean_end(&mut rig, due + 610_000);
        assert!(!rig.idle.halted);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn an_unsent_script_halts_until_the_next_phase_row() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        rig.offer(due);
        rig.idle.unsent(7, t(due), &mut rig.surface);
        assert!(rig.idle.standing.is_none());
        for n in 0..50 {
            rig.quiet(due + 1 + n * 12_000);
        }
        let refusals = rig.surface.of(IDLE_REFUSED);
        assert_eq!(refusals.len(), 1);
        assert_eq!(refusals[0]["reason"], "unsent");
        assert_eq!(refusals[0]["script_id"], 7);
        assert_eq!(rig.surface.count(IDLE_REPLACED), 0);

        let rest = clean_end(&mut rig, due + 610_000);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn an_unbuildable_script_halts_once_until_the_next_phase_row() {
        // `Playlist::parse` checks names, not durations: a clip this long puts
        // the script's timeout past what a script may carry.
        let long = MotionTable::of([
            ("long/one".to_owned(), entry(30, 600_000, 200)),
            ("long/two".to_owned(), entry(31, 600_000, 200)),
        ]);
        let playlist = Playlist::parse(r#"{"playlist":["long/one","long/two"]}"#, &long)
            .expect("names that resolve");
        let mut rig = Rig {
            idle: Idle::new(playlist, POD, SEED),
            surface: Recorded::default(),
            next_id: 1,
        };
        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            0,
        )]);

        rig.quiet(0);
        let refusals = rig.surface.of(IDLE_REFUSED);
        assert_eq!(refusals.len(), 1);
        assert_eq!(refusals[0]["reason"], "unbuildable");
        assert_eq!(refusals[0]["script_id"], Value::Null);
        assert!(
            refusals[0]["detail"]
                .as_str()
                .is_some_and(|detail| !detail.is_empty()),
            "{:?}",
            refusals[0]
        );

        rig.quiet(1);
        rig.quiet(1_000);
        rig.quiet(600_000);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 1);

        // The arm left no pending script behind to be narrated twice.
        rig.idle.refused(t(600_001), &mut rig.surface);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 1);

        rig.rows(vec![phase(
            SessionPhaseWire::RESTING,
            SessionPhaseWire::STARTING,
            700_000,
        )]);
        assert!(!rig.idle.halted);
        rig.quiet(700_000);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 2);
    }

    #[test]
    fn a_session_refusal_of_the_standing_script_halts_until_the_next_phase_row() {
        let mut rig = active_loop();
        rig.rows(vec![refused_row(1, 1, 1_500)]);
        for n in 0..50 {
            rig.quiet(1_600 + n * 12_000);
        }
        assert_eq!(rig.surface.count(IDLE_REFUSED), 1);
        let rest = clean_end(&mut rig, 610_000);
        let (_, script) = rig.send(rest);
        assert_shape(&script, OPEN_PLAY_MS);
    }

    #[test]
    fn a_refusal_naming_another_script_changes_nothing() {
        let mut rig = active_loop();
        let due = rig.standing_due();
        rig.rows(vec![refused_row(99, 1, 1_500)]);
        assert!(!rig.idle.halted);
        rig.send(due);
        assert_eq!(rig.surface.count(IDLE_REFUSED), 0);
    }

    #[test]
    fn backoff_and_park_are_narrated_once_each() {
        let mut rig = active_loop();
        let rest = fault_end(&mut rig, 6_000);
        for n in 0..10 {
            rig.quiet(rest + n * 1_000);
        }
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 1);

        rig.rows(vec![phase(
            SessionPhaseWire::PARKED,
            SessionPhaseWire::RESTING,
            rest + 10_000,
        )]);
        for n in 0..10 {
            rig.quiet(rest + 11_000 + n * 1_000);
        }
        rig.rows(vec![phase(
            SessionPhaseWire::PARKED,
            SessionPhaseWire::PARKED,
            rest + 30_000,
        )]);
        assert_eq!(rig.surface.count(IDLE_PARKED), 1);
        assert_eq!(rig.surface.count(IDLE_BACKOFF), 1);
    }

    #[test]
    fn a_resume_at_resting_opens_and_the_loop_owns_the_head() {
        let mut rig = resting_loop();
        rig.accept(&closing(8_000), 0);
        let resume = 8_000 - SEAM_LEAD_MS;
        assert_eq!(rig.quiet(10).due, Some(t(resume)));
        // The session refused speech's script; no phase row follows.
        rig.rows(vec![refused_row(5, 1, 20)]);
        rig.quiet(resume - 1);
        let (_, script) = rig.send(resume);
        assert_shape(&script, OPEN_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_OPENED), 1);
        assert_eq!(rig.surface.count(IDLE_RESUMED), 0);
        assert_eq!(rig.idle.owner, Owner::Loop);
    }

    #[test]
    fn a_closing_accepted_inside_the_lead_resumes_on_the_same_poll() {
        let mut rig = active_loop();
        rig.accept(&closing(40), 2_000);
        let (_, script) = rig.send(2_000);
        assert_shape(&script, SEAM_PLAY_MS);
        assert_eq!(rig.surface.count(IDLE_RESUMED), 1);
    }
}
