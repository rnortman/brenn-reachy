//! The edge's knobs, and the numbers they ship with.
//!
//! Two, and each is a screen the compile needs — not a policy about motion.
//! Nothing here changes what the machine decides: the session screens what this
//! crate sends and the mover checks every value it commands, whatever these
//! say. How long the schedule's closing stow takes is not among them: that is
//! the pose library's own pace for the stow, which arrives with the numbering
//! in the names sidecar.

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
    /// The largest body the edge will look at, bytes.
    body_cap_bytes: usize,
}

impl EdgeConfig {
    /// The configuration these values state.
    ///
    /// # Errors
    ///
    /// [`ConfigError`] for an unnamed pod, or a body cap no script could
    /// survive.
    pub fn new(pod: impl Into<String>, body_cap_bytes: usize) -> Result<Self, ConfigError> {
        let pod = pod.into();
        if pod.trim().is_empty() {
            return Err(ConfigError::PodUnnamed);
        }
        if body_cap_bytes < MIN_BODY_CAP_BYTES {
            return Err(ConfigError::CapBelowAnyScript { body_cap_bytes });
        }
        Ok(Self {
            pod,
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
        Self::new(pod, BODY_CAP_BYTES)
            .expect("the shipped numbers are lawful and the machine is named")
    }

    /// The name this machine answers to.
    #[must_use]
    pub fn pod(&self) -> &str {
        &self.pod
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
    use motion_proto::{MotionScript, STOW_POSE, Step};

    use super::{BODY_CAP_BYTES, ConfigError, EdgeConfig, MIN_BODY_CAP_BYTES};

    #[test]
    fn the_shipped_numbers_are_the_ones_the_screens_run_with() {
        let config = EdgeConfig::for_pod("reachy00");
        assert_eq!(config.pod(), "reachy00");
        assert_eq!(config.body_cap_bytes(), BODY_CAP_BYTES);
    }

    #[test]
    fn a_machine_with_no_name_is_not_a_configuration() {
        assert_eq!(
            EdgeConfig::new("", BODY_CAP_BYTES),
            Err(ConfigError::PodUnnamed),
        );
        assert_eq!(
            EdgeConfig::new("  \t ", BODY_CAP_BYTES),
            Err(ConfigError::PodUnnamed),
            "whitespace is a field the file left blank, not a name",
        );
    }

    #[test]
    fn a_cap_no_script_fits_under_is_not_a_configuration() {
        assert_eq!(
            EdgeConfig::new("reachy00", 0),
            Err(ConfigError::CapBelowAnyScript { body_cap_bytes: 0 }),
        );
        assert!(EdgeConfig::new("reachy00", MIN_BODY_CAP_BYTES).is_ok());
    }

    #[test]
    fn the_floor_is_above_a_script_the_wire_contract_encodes() {
        let script = MotionScript::new(
            "reachy00",
            1,
            vec![
                Step::new(0, crate::fixture::NEUTRAL_POSE),
                Step::new(2000, STOW_POSE),
            ],
            30_000,
        )
        .expect("a lawful timeline");
        assert!(
            script.encode().len() < MIN_BODY_CAP_BYTES,
            "the floor has to admit the shape a scripter publishes",
        );
    }
}
