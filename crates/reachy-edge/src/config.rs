//! The edge's knobs, and the numbers they ship with.
//!
//! Three, and each is a screen or a duration the compile needs — not a policy
//! about motion. Nothing here changes what the machine decides: the session
//! screens what this crate sends and the mover checks every value it commands,
//! whatever these say.

/// How long the stow at the end of a schedule takes, milliseconds.
///
/// The wake gesture's own stow budget, which is the only measured number
/// available: it is what the machine has been folding in since the first motion
/// run. Every schedule ends stowed — either at a stow the script itself asked
/// for, or at one the compile appends — and this is how long that step lasts.
pub const STOW_DURATION_MS: u32 = 3000;

/// The largest intent body the edge will look at, bytes.
///
/// Comfortably above any script the compile could accept — sixteen steps and
/// four overlay windows of JSON is a few hundred bytes — so a body past this is
/// not a script that will be refused later for its size but a sender doing
/// something else entirely. Applied to the bytes, before the parse: the cheap
/// screen goes first, because the expensive one is the one an unbounded body
/// makes expensive.
pub const BODY_CAP_BYTES: usize = 8192;

/// The smallest cap that is a screen rather than a gag, bytes.
///
/// A one-step script for a named pod is on the order of a hundred bytes, so a
/// cap under this one refuses every script a scripter could write and leaves the
/// machine deaf with the narration pointing at the sender. A case below pins the
/// order of magnitude against a script the wire contract actually encodes.
pub const MIN_BODY_CAP_BYTES: usize = 512;

/// What one edge is configured with.
///
/// The pod name has no default. A script is addressed, and an edge that
/// answered to whatever name a body carried would run another machine's
/// timeline on this one.
///
/// Built only through [`EdgeConfig::new`] or [`EdgeConfig::for_pod`]: these
/// values arrive from an operator's file, and each of them has settings that
/// would turn every lawful script into a refusal naming the session or the
/// sender rather than the typo that caused it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EdgeConfig {
    /// The name this machine answers to. A script addressed elsewhere is
    /// dropped and narrated.
    pod: String,
    /// How long the schedule's closing stow takes, milliseconds.
    stow_duration_ms: u32,
    /// The largest body the edge will look at, bytes.
    body_cap_bytes: usize,
}

impl EdgeConfig {
    /// The configuration these values state.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] for an unnamed pod, or a stow duration or body cap no
    /// script could survive.
    pub fn new(
        pod: impl Into<String>,
        stow_duration_ms: u32,
        body_cap_bytes: usize,
    ) -> Result<Self, ConfigError> {
        let pod = pod.into();
        if pod.trim().is_empty() {
            return Err(ConfigError::PodUnnamed);
        }
        if stow_duration_ms == 0 {
            return Err(ConfigError::StowTakesNoTime);
        }
        if u64::from(stow_duration_ms) > motion_proto::MAX_TIMEOUT_MS {
            return Err(ConfigError::StowLongerThanAnyScript { stow_duration_ms });
        }
        if body_cap_bytes < MIN_BODY_CAP_BYTES {
            return Err(ConfigError::CapBelowAnyScript { body_cap_bytes });
        }
        Ok(Self {
            pod,
            stow_duration_ms,
            body_cap_bytes,
        })
    }

    /// The shipped configuration for the pod named `pod`.
    ///
    /// # Panics
    ///
    /// If `pod` is blank, which is a caller stating no machine at all rather
    /// than an operator's file to be refused.
    #[must_use]
    pub fn for_pod(pod: impl Into<String>) -> Self {
        Self::new(pod, STOW_DURATION_MS, BODY_CAP_BYTES)
            .expect("the shipped numbers are lawful and the machine is named")
    }

    /// The name this machine answers to.
    #[must_use]
    pub fn pod(&self) -> &str {
        &self.pod
    }

    /// How long the schedule's closing stow takes, milliseconds.
    #[must_use]
    pub fn stow_duration_ms(&self) -> u32 {
        self.stow_duration_ms
    }

    /// The largest body the edge will look at, bytes.
    #[must_use]
    pub fn body_cap_bytes(&self) -> usize {
        self.body_cap_bytes
    }
}

/// Why a set of numbers is not a configuration.
///
/// Each is a value that compiles nothing: refused where it is read, so the
/// operator hears about the file rather than about every script that met it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ConfigError {
    /// A machine with no name. Every script is addressed, and a name no script
    /// can carry refuses all of them as another machine's: the operator hears
    /// the sender named for a field the file left out.
    #[error("the edge is configured with no pod name; every script is addressed")]
    PodUnnamed,

    /// A stow of no duration. The schedule's closing step would own no time,
    /// which the session refuses as a bad timeline — a refusal naming the
    /// session for a fault that is in the file.
    #[error("the stow takes no time; every schedule ends with one")]
    StowTakesNoTime,

    /// A stow longer than the longest timeout a script may state, which leaves
    /// no script any room to end in.
    #[error(
        "the stow takes {stow_duration_ms} ms, longer than the longest timeout a script may state"
    )]
    StowLongerThanAnyScript {
        /// What the file said.
        stow_duration_ms: u32,
    },

    /// A cap under the size of a script, which drops every body before the
    /// parse.
    #[error("the body cap is {body_cap_bytes} bytes, under the size of a one-step script")]
    CapBelowAnyScript {
        /// What the file said.
        body_cap_bytes: usize,
    },
}

#[cfg(test)]
mod tests {
    use motion_proto::{MotionScript, Posture, Step};

    use super::{BODY_CAP_BYTES, ConfigError, EdgeConfig, MIN_BODY_CAP_BYTES, STOW_DURATION_MS};

    #[test]
    fn the_shipped_numbers_are_the_ones_the_screens_run_with() {
        let config = EdgeConfig::for_pod("reachy00");
        assert_eq!(config.pod(), "reachy00");
        assert_eq!(config.stow_duration_ms(), STOW_DURATION_MS);
        assert_eq!(config.body_cap_bytes(), BODY_CAP_BYTES);
    }

    #[test]
    fn a_machine_with_no_name_is_not_a_configuration() {
        assert_eq!(
            EdgeConfig::new("", STOW_DURATION_MS, BODY_CAP_BYTES),
            Err(ConfigError::PodUnnamed),
        );
        assert_eq!(
            EdgeConfig::new("  \t ", STOW_DURATION_MS, BODY_CAP_BYTES),
            Err(ConfigError::PodUnnamed),
            "whitespace is a field the file left blank, not a name",
        );
    }

    #[test]
    fn a_stow_of_no_duration_is_not_a_configuration() {
        assert_eq!(
            EdgeConfig::new("reachy00", 0, BODY_CAP_BYTES),
            Err(ConfigError::StowTakesNoTime),
        );
        assert_eq!(
            EdgeConfig::new("reachy00", 600_001, BODY_CAP_BYTES),
            Err(ConfigError::StowLongerThanAnyScript {
                stow_duration_ms: 600_001
            }),
        );
    }

    #[test]
    fn a_cap_no_script_fits_under_is_not_a_configuration() {
        assert_eq!(
            EdgeConfig::new("reachy00", STOW_DURATION_MS, 0),
            Err(ConfigError::CapBelowAnyScript { body_cap_bytes: 0 }),
        );
        assert!(EdgeConfig::new("reachy00", STOW_DURATION_MS, MIN_BODY_CAP_BYTES).is_ok());
    }

    #[test]
    fn the_floor_is_above_a_script_the_wire_contract_encodes() {
        let script = MotionScript::new(
            "reachy00",
            1,
            vec![Step::new(0, Posture::Up), Step::new(2000, Posture::Stow)],
            30_000,
        )
        .expect("a lawful timeline");
        assert!(
            script.encode().len() < MIN_BODY_CAP_BYTES,
            "the floor has to admit the shape a scripter publishes",
        );
    }
}
