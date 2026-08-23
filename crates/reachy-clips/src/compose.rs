//! The compositor: a base target, plus whatever is playing over it.
//!
//! One pure function, called once per tick. It takes the target something else
//! decided — a held posture, a posture transition's sample, later a tracker's
//! stream — and folds each playing overlay's masked, weighted delta onto it.
//! The compositor never asks who produced the base, which is what makes the
//! base a seam rather than a hard-coded posture timeline.
//!
//! Two rules carry the meaning:
//!
//! - **Masking is exact.** A channel no overlay drives comes out of the fold
//!   bit-identical to the base. An antennas-only wiggle over a held head leaves
//!   the head command unchanged, not approximately unchanged.
//! - **The head delta is applied in the base head's own frame** — a right
//!   multiplication — so a nod recorded at neutral nods a head the base is
//!   pointing somewhere else, instead of dragging it back to where the
//!   recording was made. Body yaw and antennas are scalars and simply add.
//!
//! Overlays fold in the order the caller passes them, which is wire step order.
//! Two head overlays composed in the other order give a different pose, because
//! rotations do not commute; the order is fixed and deterministic rather than
//! pretended away.
//!
//! **Nothing here is a safety check.** A delta that was valid over the base it
//! was recorded against can leave the envelope over an aggressive one. What
//! this function returns faces the per-tick envelope check and step bound like
//! any other commanded target, and that check is the gate.

use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use reachy_motion::JointTargets;

use crate::format::{Channel, ChannelMask, DeltaFrame, PerChannel};

/// How strongly each channel of one overlay contributes this tick.
///
/// Per channel rather than one scalar per overlay because a motion's segments
/// need not all drive the same channels: when a head-and-antennas clip is
/// followed by an antennas-only one, the head fades out on the outgoing clip's
/// ramp while the antennas carry on at full weight. A single weight could not
/// express that instant.
///
/// Weights are in `[0, 1]`. A player never emits anything else; a caller that
/// synthesises one — a future modulation source driving an overlay's weight
/// from speech amplitude — is expected to hold the same range, since the
/// blend-floor derivation that keeps a ramp inside the step bounds assumes it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChannelWeights(PerChannel<f64>);

impl ChannelWeights {
    /// Every channel silent: what an overlay contributes before its blend-in
    /// and after its blend-out.
    #[must_use]
    pub const fn zero() -> Self {
        Self(PerChannel::new([0.0; Channel::COUNT]))
    }

    /// Every channel at full weight.
    #[must_use]
    pub const fn full() -> Self {
        Self(PerChannel::new([1.0; Channel::COUNT]))
    }

    /// The weight on `channel`.
    #[must_use]
    pub const fn get(self, channel: Channel) -> f64 {
        *self.0.get(channel)
    }

    /// Set the weight on `channel`.
    pub fn set(&mut self, channel: Channel, weight: f64) {
        self.0.set(channel, weight);
    }

    /// The channels carrying any weight at all.
    #[must_use]
    pub fn mask(self) -> ChannelMask {
        let mut mask = ChannelMask::empty();
        for channel in Channel::ALL {
            if self.get(channel) > 0.0 {
                mask.insert(channel);
            }
        }
        mask
    }
}

/// One overlay's contribution for one tick: its deltas and their weights.
///
/// A channel contributes only when the frame carries a delta for it *and* its
/// weight is above zero; the two agree by construction when a player produced
/// the sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OverlaySample {
    /// The deltas, keyed by channel.
    pub frame: DeltaFrame,
    /// How strongly each one applies.
    pub weights: ChannelWeights,
}

impl OverlaySample {
    /// A sample that changes nothing, for a player that is between segments.
    #[must_use]
    pub fn silent() -> Self {
        Self {
            frame: DeltaFrame::zero(ChannelMask::empty()),
            weights: ChannelWeights::zero(),
        }
    }
}

/// Fold `overlays` onto `base` and return the target to command.
///
/// Channels no overlay drives are copied from the base untouched.
#[must_use]
pub fn compose(base: JointTargets, overlays: &[OverlaySample]) -> JointTargets {
    let mut out = base;
    for sample in overlays {
        if let Some(delta) = sample.frame.head {
            let weight = sample.weights.get(Channel::Head);
            if weight > 0.0 {
                out.head_pose_body *= scale_delta(&delta, weight);
            }
        }
        if let Some(delta) = sample.frame.body_yaw {
            out.body_yaw += delta * sample.weights.get(Channel::BodyYaw);
        }
        if let Some(delta) = sample.frame.antennas {
            let weight = sample.weights.get(Channel::Antennas);
            out.antennas[0] += delta[0] * weight;
            out.antennas[1] += delta[1] * weight;
        }
    }
    out
}

/// The base `composed` was folded onto, given the overlays that were riding it.
///
/// [`compose`] run backwards, over the same arithmetic and in the reverse order,
/// so the two cannot come to disagree about what a fold took out.
///
/// What a caller needs it for: an overlay that stops riding a base takes its
/// whole weighted delta out of the composed stream in one period unless the base
/// absorbs it. Re-anchoring the base at this — the composed setpoint that was
/// last commanded, less what still rides it — makes the next composed setpoint
/// the same setpoint, and the offset then decays as a planned move under
/// whatever bounds the caller's own commands are held to.
///
/// Exact for the scalars and to the arithmetic's own precision for the head:
/// the rotation is unwound along the same arc it was wound along.
#[must_use]
pub fn uncompose(composed: JointTargets, overlays: &[OverlaySample]) -> JointTargets {
    let mut out = composed;
    for sample in overlays.iter().rev() {
        if let Some(delta) = sample.frame.head {
            let weight = sample.weights.get(Channel::Head);
            if weight > 0.0 {
                out.head_pose_body *= scale_delta(&delta, weight).inverse();
            }
        }
        if let Some(delta) = sample.frame.body_yaw {
            out.body_yaw -= delta * sample.weights.get(Channel::BodyYaw);
        }
        if let Some(delta) = sample.frame.antennas {
            let weight = sample.weights.get(Channel::Antennas);
            out.antennas[0] -= delta[0] * weight;
            out.antennas[1] -= delta[1] * weight;
        }
    }
    out
}

/// A rigid delta scaled by `weight`: the same motion, `weight` of the way
/// through it.
///
/// Scaling a delta is interpolating from the identity toward it, and it is
/// written as exactly that so a weight ramp and the player's frame
/// interpolation cannot come to disagree about what half of a rotation is.
#[must_use]
pub fn scale_delta(delta: &Isometry3<f64>, weight: f64) -> Isometry3<f64> {
    interpolate_pose(&Isometry3::identity(), delta, weight)
}

/// Interpolate a rigid transform: straight line on the translation, shortest
/// arc on the rotation.
///
/// The one place in this crate a rotation is bent — the compositor's weight
/// scaling and the player's between-frame sampling both come here — so a change
/// to the convention (a slerp, a guard near half a turn) lands once. Coupled to
/// `Trajectory::sample` in `reachy-motion`, which must walk the same arc so a
/// delta and a planned move take the same path between two orientations.
#[must_use]
pub fn interpolate_pose(from: &Isometry3<f64>, to: &Isometry3<f64>, s: f64) -> Isometry3<f64> {
    let start = from.translation.vector;
    let rotvec = (from.rotation.inverse() * to.rotation).scaled_axis();
    Isometry3::from_parts(
        Translation3::from(start + (to.translation.vector - start) * s),
        from.rotation * UnitQuaternion::from_scaled_axis(rotvec * s),
    )
}

/// `from` at `s = 0`, `to` at `s = 1`.
#[must_use]
pub fn lerp(from: f64, to: f64, s: f64) -> f64 {
    from + (to - from) * s
}

#[cfg(test)]
mod tests {
    use super::*;

    use core::f64::consts::FRAC_PI_2;
    use nalgebra::Vector3;

    /// A head-only sample rotating by `angle` about the body-frame z axis.
    fn head_yaw_sample(angle: f64, weight: f64) -> OverlaySample {
        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Head, weight);
        OverlaySample {
            frame: DeltaFrame {
                head: Some(Isometry3::from_parts(
                    Translation3::new(0.0, 0.0, 0.0),
                    UnitQuaternion::from_scaled_axis(Vector3::z() * angle),
                )),
                antennas: None,
                body_yaw: None,
            },
            weights,
        }
    }

    fn antennas_sample(right: f64, left: f64, weight: f64) -> OverlaySample {
        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Antennas, weight);
        OverlaySample {
            frame: DeltaFrame {
                head: None,
                antennas: Some([right, left]),
                body_yaw: None,
            },
            weights,
        }
    }

    /// A base that is not the neutral one, so a passthrough test can tell the
    /// difference between "unchanged" and "reset to neutral".
    fn moved_base() -> JointTargets {
        let mut base = JointTargets::default();
        base.head_pose_body *= Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.01),
            UnitQuaternion::from_scaled_axis(Vector3::y() * 0.2),
        );
        base.body_yaw = 0.4;
        base.antennas = [0.3, -0.3];
        base
    }

    #[test]
    fn no_overlays_returns_the_base_untouched() {
        let base = moved_base();
        assert_eq!(compose(base, &[]), base);
    }

    #[test]
    fn unmasked_channels_are_bit_identical_to_the_base() {
        let base = moved_base();
        let out = compose(base, &[antennas_sample(0.1, -0.1, 1.0)]);
        assert_eq!(out.head_pose_body, base.head_pose_body);
        assert_eq!(out.body_yaw, base.body_yaw);
        assert_ne!(out.antennas, base.antennas);
    }

    #[test]
    fn antenna_and_yaw_deltas_add_to_the_base() {
        let base = moved_base();
        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Antennas, 1.0);
        weights.set(Channel::BodyYaw, 1.0);
        let sample = OverlaySample {
            frame: DeltaFrame {
                head: None,
                antennas: Some([0.1, -0.2]),
                body_yaw: Some(0.05),
            },
            weights,
        };
        let out = compose(base, &[sample]);
        assert!((out.body_yaw - 0.45).abs() < 1e-12);
        assert!((out.antennas[0] - 0.4).abs() < 1e-12);
        assert!((out.antennas[1] + 0.5).abs() < 1e-12);
    }

    #[test]
    fn scalar_weights_scale_linearly() {
        let base = JointTargets::default();
        let out = compose(base, &[antennas_sample(0.4, -0.4, 0.25)]);
        assert!((out.antennas[0] - 0.1).abs() < 1e-12);
        assert!((out.antennas[1] + 0.1).abs() < 1e-12);
    }

    #[test]
    fn zero_weight_leaves_the_base_exactly() {
        let base = moved_base();
        let out = compose(
            base,
            &[
                head_yaw_sample(FRAC_PI_2, 0.0),
                antennas_sample(1.0, 1.0, 0.0),
            ],
        );
        assert_eq!(out, base);
    }

    #[test]
    fn head_delta_applies_in_the_base_head_frame() {
        // A base yawed a quarter turn about z, plus a delta that translates
        // along the head's own x. Right multiplication moves along the base's
        // rotated x, which in the body frame is y.
        let mut base = JointTargets::default();
        let quarter = UnitQuaternion::from_scaled_axis(Vector3::z() * FRAC_PI_2);
        base.head_pose_body *= Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), quarter);

        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Head, 1.0);
        let sample = OverlaySample {
            frame: DeltaFrame {
                head: Some(Isometry3::from_parts(
                    Translation3::new(0.02, 0.0, 0.0),
                    UnitQuaternion::identity(),
                )),
                antennas: None,
                body_yaw: None,
            },
            weights,
        };
        let out = compose(base, &[sample]);
        let moved = out.head_pose_body.translation.vector - base.head_pose_body.translation.vector;
        assert!(moved.x.abs() < 1e-12);
        assert!((moved.y - 0.02).abs() < 1e-12);
        assert!(moved.z.abs() < 1e-12);
    }

    #[test]
    fn head_weight_scales_the_rotation_along_its_rotvec() {
        let base = JointTargets::default();
        let out = compose(base, &[head_yaw_sample(FRAC_PI_2, 0.5)]);
        let relative = base.head_pose_body.rotation.inverse() * out.head_pose_body.rotation;
        assert!((relative.angle() - FRAC_PI_2 / 2.0).abs() < 1e-12);
    }

    #[test]
    fn a_partial_head_weight_scales_the_translation_too() {
        // A delta carrying both a translation and a rotation, at a quarter
        // weight: the whole rigid delta is scaled, so the head moves a quarter
        // of the way along it rather than jumping the translation the instant
        // the weight leaves zero.
        let base = moved_base();
        let delta = Isometry3::from_parts(
            Translation3::new(0.02, -0.01, 0.03),
            UnitQuaternion::from_scaled_axis(Vector3::y() * 0.4),
        );
        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Head, 0.25);
        let sample = OverlaySample {
            frame: DeltaFrame {
                head: Some(delta),
                antennas: None,
                body_yaw: None,
            },
            weights,
        };

        let out = compose(base, &[sample]);
        let expected = base.head_pose_body * interpolate_pose(&Isometry3::identity(), &delta, 0.25);
        assert!(
            (out.head_pose_body.translation.vector - expected.translation.vector).norm() < 1e-12
        );
        assert!(out.head_pose_body.rotation.angle_to(&expected.rotation) < 1e-12);

        // And that is a quarter of the delta's own translation, in the base
        // head's frame: at full weight the head moves four times as far.
        let quarter =
            out.head_pose_body.translation.vector - base.head_pose_body.translation.vector;
        let full = compose(
            base,
            &[OverlaySample {
                frame: sample.frame,
                weights: ChannelWeights::full(),
            }],
        );
        let whole = full.head_pose_body.translation.vector - base.head_pose_body.translation.vector;
        assert!((quarter * 4.0 - whole).norm() < 1e-12);
    }

    #[test]
    fn full_head_weight_is_the_whole_delta() {
        let base = moved_base();
        let out = compose(base, &[head_yaw_sample(0.3, 1.0)]);
        let relative = base.head_pose_body.rotation.inverse() * out.head_pose_body.rotation;
        assert!((relative.angle() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn head_overlays_fold_in_the_order_given() {
        let base = JointTargets::default();
        let mut pitch = head_yaw_sample(0.0, 1.0);
        pitch.frame.head = Some(Isometry3::from_parts(
            Translation3::new(0.03, 0.0, 0.0),
            UnitQuaternion::from_scaled_axis(Vector3::y() * 0.5),
        ));
        let yaw = head_yaw_sample(0.7, 1.0);

        let one = compose(base, &[pitch, yaw]);
        let other = compose(base, &[yaw, pitch]);
        assert_ne!(one.head_pose_body, other.head_pose_body);

        // Order is fixed, not merely arbitrary: at full weight the fold is the
        // base right-multiplied by each delta in the order given.
        let expected = base.head_pose_body
            * pitch.frame.head.expect("head delta")
            * yaw.frame.head.expect("head delta");
        let relative = expected.inverse() * one.head_pose_body;
        assert!(relative.rotation.angle() < 1e-12, "{relative}");
        assert!(relative.translation.vector.norm() < 1e-12, "{relative}");
    }

    #[test]
    fn scalar_overlays_are_order_independent_and_additive() {
        let base = moved_base();
        let first = antennas_sample(0.1, 0.2, 1.0);
        let second = antennas_sample(-0.05, 0.4, 0.5);
        let one = compose(base, &[first, second]);
        let other = compose(base, &[second, first]);
        assert!((one.antennas[0] - other.antennas[0]).abs() < 1e-15);
        assert!((one.antennas[1] - other.antennas[1]).abs() < 1e-15);
        assert!((one.antennas[0] - (0.3 + 0.1 - 0.025)).abs() < 1e-12);
    }

    #[test]
    fn a_silent_sample_changes_nothing() {
        let base = moved_base();
        assert_eq!(compose(base, &[OverlaySample::silent()]), base);
    }

    /// The fold is undone by the same arithmetic that made it: a base that goes
    /// through both comes back out, whatever was riding it and in whatever
    /// order. What a caller re-anchoring a base off a composed setpoint relies
    /// on, and the reason head deltas are unwound in the reverse order.
    #[test]
    fn uncomposing_a_setpoint_hands_back_the_base_it_stood_on() {
        let base = moved_base();
        let mut pitch = head_yaw_sample(0.0, 0.75);
        pitch.frame.head = Some(Isometry3::from_parts(
            Translation3::new(0.03, -0.01, 0.0),
            UnitQuaternion::from_scaled_axis(Vector3::y() * 0.5),
        ));
        let mut both = antennas_sample(0.1, -0.2, 0.5);
        both.frame.body_yaw = Some(0.07);
        both.weights.set(Channel::BodyYaw, 0.25);
        let riding = [pitch, head_yaw_sample(0.7, 1.0), both];

        for overlays in [&riding[..], &riding[..1], &riding[2..], &[][..]] {
            let composed = compose(base, overlays);
            let found = uncompose(composed, overlays);
            assert!(
                (found.head_pose_body.translation.vector - base.head_pose_body.translation.vector)
                    .norm()
                    < 1e-12,
                "{found:?}"
            );
            assert!(
                found
                    .head_pose_body
                    .rotation
                    .angle_to(&base.head_pose_body.rotation)
                    < 1e-12
            );
            assert!((found.body_yaw - base.body_yaw).abs() < 1e-12);
            assert!((found.antennas[0] - base.antennas[0]).abs() < 1e-12);
            assert!((found.antennas[1] - base.antennas[1]).abs() < 1e-12);
        }

        // And it is the fold's inverse rather than a no-op: a setpoint nothing
        // was riding is the base, and one something was is not.
        assert_ne!(compose(base, &riding), base);
        assert_eq!(uncompose(base, &[]), base);
    }

    #[test]
    fn weights_report_the_channels_they_drive() {
        let mut weights = ChannelWeights::zero();
        weights.set(Channel::Antennas, 0.5);
        let mask = weights.mask();
        assert!(mask.contains(Channel::Antennas));
        assert!(!mask.contains(Channel::Head));
        assert!(!mask.contains(Channel::BodyYaw));
        assert_eq!(ChannelWeights::full().mask().iter().count(), 3);
        assert!(ChannelWeights::zero().mask().is_empty());
    }
}
