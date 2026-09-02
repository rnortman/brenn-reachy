//! The intake: one gate, both sources.
//!
//! Intent reaches this machine two ways — handed over in-process by the
//! scripter that authored it, and delivered off the bus from a remote sender —
//! and both meet the same screens here. One gate because one sender authority
//! per pod at a time is the presence protocol's own model: interleaved senders
//! are a configuration error, and the sequence gate turns that error into
//! narrated drops rather than into two timelines fighting over one head.
//!
//! The gate is phase-blind, deliberately. This crate holds a copy of the
//! session's latest narration, and that copy is an observation of what the
//! machine last *said*, possibly stale by the time it is read. A screen resting
//! on it would gate torque decisions on second-hand state that only the session
//! speaks for. So nothing here asks what the machine is doing; the session
//! refuses what it will not accept, and says so on its own narration.
//!
//! Nothing is queued and nothing is retried. The latest script forwards, the
//! session's latest-wins does the rest, and a sender's refresh cadence is its
//! liveness — a retry from here would be a second engagement nobody asked for.

use clockwork_rs::{SyncTime, blob_as_bytes};
use motion_proto::{DecodeError, MotionScript};
use thiserror::Error;

use brenn_reachy__cogs__script_clk_rs::ScriptWire;

use crate::compile::{CompileError, compile};
use crate::config::EdgeConfig;
use crate::names::MotionTable;

/// The intent edge: the screens, the numbering, and what they were configured
/// with.
///
/// One per process. Holding the sequence high-water mark and the script counter
/// together is what makes the two intent sources one authority; two edges would
/// be two counters, and the session would see one sender's numbering jump
/// backwards.
#[derive(Debug)]
pub struct Edge {
    config: EdgeConfig,
    table: MotionTable,
    /// The highest sequence number accepted this run, or `None` before the
    /// first. Nothing persists it: a restarted host counts from nothing, and
    /// the running schedule's own horizon is what concludes the engagement it
    /// can no longer refresh.
    accepted_seq: Option<u64>,
    /// How many scripts this run has issued, which is also the last id it used.
    issued: u32,
}

/// Where a body offered to the gate was authored.
///
/// The gate itself does not read this — every screen applies the same way to
/// both sources, which is what makes one gate one authority. What reads it is
/// the alert table: a refusal of a script *this machine wrote for itself* is
/// the machine disagreeing with itself, and no sender's next refresh will ever
/// resolve it, whereas a refusal of somebody else's script is the disagreement
/// the channel is expected to carry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    /// Authored on this machine, for this machine: the pipeline's scripter, or
    /// the motion harness's pinned gesture.
    Local,
    /// Delivered off the bus by a sender that is not this process.
    Remote,
}

/// A script the edge accepted, ready to send.
#[derive(Debug, PartialEq)]
pub struct Accepted {
    /// The id this edge gave it, which the session's narration reports back.
    pub script_id: u32,
    /// The sequence number the sender numbered it with.
    pub seq: u64,
    /// The request itself.
    pub message: ScriptWire,
}

impl Accepted {
    /// The message as the datagram carries it: the schema's own bytes, no
    /// header and no length, because the port is what says which schema it is.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        blob_as_bytes(&self.message)
    }
}

/// Why a body did not become a request.
///
/// Each is a drop, narrated, and none of them stops the edge: a
/// screen that took the process down would let one bad body cost the machine
/// its voice, and the script already running — with its own timeout — stands.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum Refusal {
    /// A body past the configured cap, refused before anything parsed it.
    #[error("the body is {bytes} bytes; the edge reads at most {cap} bytes")]
    Oversize {
        /// How big it was.
        bytes: usize,
        /// The cap it passed.
        cap: usize,
    },

    /// A body that is not text at all. Separate from a decode refusal because
    /// it says something different about the sender: not a scripter with a bug
    /// but something that is not a scripter.
    #[error("the body is not utf-8 text")]
    NotText,

    /// The text is not a script this build can run.
    #[error("{0}")]
    Undecodable(#[from] DecodeError),

    /// A script addressed to another machine. Dropped rather than run: the
    /// channel is not assumed to carry one pod's traffic, and a machine that
    /// answered to any name would run another's timeline.
    #[error("the script is addressed to `{addressed}`; this machine is `{pod}`")]
    ForeignPod {
        /// Whose script it is.
        addressed: String,
        /// Whose machine this is.
        pod: String,
    },

    /// A redelivery: numbered at or below one already accepted this run. This
    /// is the stale-delivery defence the session at rest deliberately does not
    /// perform, landing at the edge that received the delivery.
    #[error("the script is numbered {seq}, at or below the accepted {accepted}")]
    Stale {
        /// What arrived.
        seq: u64,
        /// The high-water mark it did not pass.
        accepted: u64,
    },

    /// The script decoded and does not become a lawful schedule.
    #[error("{0}")]
    Uncompilable(#[from] CompileError),
}

impl Refusal {
    /// The refusal's name, as a narration line spells it.
    ///
    /// A stable word per screen, defined beside the screens rather than at each
    /// surface that reports one: a log whose spelling drifts stops joining
    /// against the runs that came before it, silently.
    #[must_use]
    pub const fn kind(&self) -> &'static str {
        match self {
            Self::Oversize { .. } => "oversize",
            Self::NotText => "not_text",
            Self::Undecodable(_) => "undecodable",
            Self::ForeignPod { .. } => "foreign_pod",
            Self::Stale { .. } => "stale",
            Self::Uncompilable(_) => "uncompilable",
        }
    }
}

impl Edge {
    /// An edge configured by `config`, resolving names through `table`.
    #[must_use]
    pub fn new(config: EdgeConfig, table: MotionTable) -> Self {
        Self {
            config,
            table,
            accepted_seq: None,
            issued: 0,
        }
    }

    /// What this edge was configured with.
    #[must_use]
    pub fn config(&self) -> &EdgeConfig {
        &self.config
    }

    /// How many scripts this run has accepted.
    #[must_use]
    pub fn issued(&self) -> u32 {
        self.issued
    }

    /// Screen `body`, received at `arrival`, and compile what survives.
    ///
    /// `arrival` is this machine's clock read at receipt, never a sender's
    /// stamp: the offsets in a script are measured from the instant it landed,
    /// and a stamp made on another clock would silently reinterpret them.
    ///
    /// # Errors
    ///
    /// [`Refusal`] for a body that does not become a request. Nothing is
    /// retried and nothing partial is sent.
    pub fn accept(&mut self, body: &[u8], arrival: SyncTime) -> Result<Accepted, Refusal> {
        if body.len() > self.config.body_cap_bytes() {
            return Err(Refusal::Oversize {
                bytes: body.len(),
                cap: self.config.body_cap_bytes(),
            });
        }
        let text = std::str::from_utf8(body).map_err(|_| Refusal::NotText)?;
        let script = MotionScript::decode(text)?;
        if script.pod() != self.config.pod() {
            return Err(Refusal::ForeignPod {
                addressed: script.pod().to_owned(),
                pod: self.config.pod().to_owned(),
            });
        }
        if let Some(accepted) = self.accepted_seq
            && script.seq() <= accepted
        {
            return Err(Refusal::Stale {
                seq: script.seq(),
                accepted,
            });
        }
        // The id is issued only for a script that compiled: a number spent on a
        // refusal would leave a gap in the sequence the session narrates back,
        // and a reader of that narration would be looking for a script nothing
        // ever sent.
        let next_id = next_id(self.issued);
        let message = compile(&script, arrival, next_id, &self.config, &self.table)?;
        self.issued = next_id;
        self.accepted_seq = Some(script.seq());
        Ok(Accepted {
            script_id: next_id,
            seq: script.seq(),
            message,
        })
    }
}

/// The id after `issued`.
///
/// Counted from one, so a zero in a report is a field nobody set rather than
/// the first script of the run. The wrap is unreachable in operation — four
/// billion scripts at the presence refresh cadence is centuries — and wrapping
/// rather than saturating is what keeps ids distinct if it ever happens.
const fn next_id(issued: u32) -> u32 {
    match issued.wrapping_add(1) {
        0 => 1,
        id => id,
    }
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__script_clk_rs::ScriptWire;
    use clockwork_rs::{Blob as _, SyncTime, blob_from_bytes};
    use motion_proto::{DecodeError, MotionScript, Posture, Step};

    use super::{Edge, Refusal, next_id};
    use crate::compile::CompileError;
    use crate::config::EdgeConfig;
    use crate::names::MotionTable;

    /// A round instant, so a stamp read off the wrong side of the intake shows.
    const ARRIVAL_NS: i64 = 1_700_000_000_000_000_000;

    /// The clock this edge is told the time by. Injected rather than read: the
    /// crate has none of its own, and the cases are the reason that is testable.
    fn at(offset_ns: i64) -> SyncTime {
        SyncTime::from_nanos(ARRIVAL_NS + offset_ns)
    }

    /// An edge for `reachy00` over an empty library: no case here plays a
    /// motion, and the compile's own suite covers the ones that do.
    fn edge() -> Edge {
        Edge::new(EdgeConfig::for_pod("reachy00"), MotionTable::default())
    }

    /// The body a scripter would publish: up now, stowed at the close.
    fn body(pod: &str, seq: u64) -> Vec<u8> {
        let script = MotionScript::new(
            pod,
            seq,
            vec![Step::new(0, Posture::Up), Step::new(2000, Posture::Stow)],
            30_000,
        )
        .expect("a lawful timeline");
        script.encode().into_bytes()
    }

    #[test]
    fn a_script_for_this_machine_compiles_stamped_and_numbered_from_one() {
        let mut edge = edge();
        let first = edge
            .accept(&body("reachy00", 100), at(0))
            .expect("a lawful script");
        assert_eq!(first.script_id, 1);
        assert_eq!(first.seq, 100);
        assert_eq!(first.message.arrival(), at(0));
        let sent = blob_from_bytes::<ScriptWire>(first.bytes())
            .expect("the datagram is the message, whole and unwrapped");
        assert_eq!(sent, first.message);
        assert_eq!(
            first.bytes().len(),
            ScriptWire::SIZE,
            "the port says which schema it is, so the bytes are the blob and nothing else",
        );

        let second = edge
            .accept(&body("reachy00", 101), at(5_000_000_000))
            .expect("a lawful script");
        assert_eq!(
            second.script_id, 2,
            "the counter climbs by one per acceptance"
        );
        assert_eq!(
            second.message.arrival(),
            at(5_000_000_000),
            "the stamp is the instant the body landed, never the sender's",
        );
        assert_eq!(edge.issued(), 2);
    }

    #[test]
    fn a_redelivery_is_dropped_whichever_source_carried_it() {
        let mut edge = edge();
        edge.accept(&body("reachy00", 100), at(0))
            .expect("the first");
        // The two intent sources share this one gate: a bus delivery numbered
        // below what the in-process scripter already had accepted is the
        // interleave the presence protocol calls a configuration error.
        assert_eq!(
            edge.accept(&body("reachy00", 100), at(1)),
            Err(Refusal::Stale {
                seq: 100,
                accepted: 100
            }),
        );
        assert_eq!(
            edge.accept(&body("reachy00", 99), at(2)),
            Err(Refusal::Stale {
                seq: 99,
                accepted: 100
            }),
        );
        assert!(edge.accept(&body("reachy00", 101), at(3)).is_ok());
        assert_eq!(edge.issued(), 2, "a drop spends no script id");
    }

    #[test]
    fn a_refused_script_does_not_advance_the_gate() {
        let mut edge = edge();
        // A timeline with no room for its closing stow: refused at the compile,
        // after the sequence gate has already looked at it.
        let unstowable = MotionScript::new("reachy00", 100, vec![Step::new(0, Posture::Up)], 2000)
            .expect("a lawful timeline");
        let unstowable = unstowable.encode().into_bytes();
        assert!(matches!(
            edge.accept(&unstowable, at(0)),
            Err(Refusal::Uncompilable(CompileError::NoRoomForStow { .. })),
        ));
        assert_eq!(edge.issued(), 0);
        assert!(
            edge.accept(&body("reachy00", 100), at(1)).is_ok(),
            "the mark moves on acceptance, so the sender's next attempt at 100 is heard",
        );
    }

    #[test]
    fn a_script_addressed_elsewhere_is_dropped() {
        let mut edge = edge();
        assert_eq!(
            edge.accept(&body("kitchen-pod", 100), at(0)),
            Err(Refusal::ForeignPod {
                addressed: "kitchen-pod".to_owned(),
                pod: "reachy00".to_owned(),
            }),
        );
    }

    #[test]
    fn a_body_past_the_cap_is_dropped_before_anything_parses_it() {
        let mut edge = edge();
        let cap = edge.config().body_cap_bytes();
        let oversize = vec![b'{'; cap + 1];
        assert_eq!(
            edge.accept(&oversize, at(0)),
            Err(Refusal::Oversize {
                bytes: cap + 1,
                cap
            }),
        );
    }

    #[test]
    fn a_body_that_is_not_a_script_is_dropped_and_named() {
        let mut edge = edge();
        assert_eq!(edge.accept(&[0xff, 0xfe], at(0)), Err(Refusal::NotText));
        assert!(matches!(
            edge.accept(b"{", at(0)),
            Err(Refusal::Undecodable(DecodeError::NotJson { .. })),
        ));
        assert!(matches!(
            edge.accept(br#"{"type": "weather", "pod": "reachy00"}"#, at(0)),
            Err(Refusal::Undecodable(DecodeError::WrongType { .. })),
        ));
    }

    #[test]
    fn the_counter_starts_at_one_and_never_reaches_zero() {
        assert_eq!(next_id(0), 1);
        assert_eq!(next_id(7), 8);
        assert_eq!(
            next_id(u32::MAX),
            1,
            "zero in a report is a field nobody set, so the wrap skips it",
        );
    }

    #[test]
    fn every_screen_has_a_word_of_its_own() {
        assert_eq!(Refusal::Oversize { bytes: 1, cap: 0 }.kind(), "oversize");
        assert_eq!(Refusal::NotText.kind(), "not_text");
        assert_eq!(
            Refusal::Undecodable(DecodeError::NotJson {
                detail: String::new(),
            })
            .kind(),
            "undecodable",
        );
        assert_eq!(
            Refusal::ForeignPod {
                addressed: String::new(),
                pod: String::new(),
            }
            .kind(),
            "foreign_pod",
        );
        assert_eq!(
            Refusal::Stale {
                seq: 0,
                accepted: 0,
            }
            .kind(),
            "stale",
        );
        assert_eq!(
            Refusal::Uncompilable(CompileError::NoPosture).kind(),
            "uncompilable",
        );

        let mut kinds = vec![
            Refusal::Oversize { bytes: 1, cap: 0 }.kind(),
            Refusal::NotText.kind(),
            Refusal::Undecodable(DecodeError::NotJson {
                detail: String::new(),
            })
            .kind(),
            Refusal::ForeignPod {
                addressed: String::new(),
                pod: String::new(),
            }
            .kind(),
            Refusal::Stale {
                seq: 0,
                accepted: 0,
            }
            .kind(),
            Refusal::Uncompilable(CompileError::NoPosture).kind(),
        ];
        kinds.sort_unstable();
        let named = kinds.len();
        kinds.dedup();
        assert_eq!(kinds.len(), named, "a narration line names one screen");
    }
}
