//! `reachy-kin` — the head's kinematics, and the envelope that keeps it intact.
//!
//! The head rides a six-legged parallel platform: six cranks, six rods, one
//! moving plate. Inverse kinematics is closed form per leg because the mechanism
//! forces it — each leg is one loop-closure equation in one crank angle. Forward
//! kinematics has no closed form and is solved numerically.
//!
//! Pure math. No I/O, no clock, no logging, and no interior state: every entry
//! point takes configuration and inputs by reference and writes its result into
//! a caller-provided output. In particular the forward solve takes its **seed as
//! an argument** rather than caching a last-known pose, so a caller in a control
//! loop owns the seed hygiene and no hidden state can make two identical calls
//! answer differently.
//!
//! Why this crate is written defensively rather than as a maths library:
//!
//! - The platform's vertical travel ends a couple of millimetres short of
//!   configurations where the linkage goes singular and loses control of the
//!   plate. Off the vertical axis that clearance is not bounded away from zero at
//!   all. The per-leg travel windows are the only thing holding the mechanism off
//!   those configurations, so the windows are enforced here, on the command path,
//!   not left in a description file that nothing reads.
//! - The clearance itself is computed per pose, as the distance from the leg's
//!   actual configuration to the one where its two solution branches merge. It is
//!   never read from a table, because the tabulated numbers describe pure vertical
//!   travel and are not bounds anywhere else.
//! - An unreachable pose is a typed error naming the leg and the shortfall. Never
//!   a clamp, never a saturated angle, never a non-finite number handed onward.
//! - The forward solve either converges to a pose that passes a plausibility
//!   screen, or returns an error. It never returns a partial pose, never returns
//!   the previous one, and never retries with perturbed inputs — the mechanism is
//!   multi-valued across assembly modes, so a converged-but-implausible answer is
//!   a wrong answer, not a near miss.
//!
//! Geometry and limits are configuration structs with defaults baked in, not
//! constants, because the dimensions of an individual unit ultimately have to be
//! fitted and swapped in as measured values.

#![forbid(unsafe_code)]
