//! The compositor: a base target, plus whatever overlay and posed clips are
//! playing over it.
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
//! - **Overlay head deltas are applied in the base head's own frame** — a right
//!   multiplication — while posed channels blend toward their authored
//!   absolute targets. Body yaw and antennas follow the same distinction; a
//!   posed antenna anchor arrives already lifted into the player's chosen turn.
//!
//! For each channel, posed samples fold first in row order and unposed samples
//! fold afterward in row order. Row order remains significant only among
//! samples of the same kind, including noncommuting unposed head rotations.
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
/// One clip contribution for one tick: its deltas, weights, and posed anchors.
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
    /// The absolute authored base for each channel, when its driver is posed.
    pub anchors: OverlayAnchors,
}

/// Absolute provenance for the three channels of one overlay sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OverlayAnchors {
    /// The authored head pose, when the head driver is posed.
    pub head: Option<Isometry3<f64>>,
    /// The authored body yaw, when the yaw driver is posed.
    pub body_yaw: Option<f64>,
    /// The authored antenna angles, when the antenna driver is posed.
    pub antennas: Option<[f64; 2]>,
}

impl OverlayAnchors {
    /// No channel has authored provenance.
    #[must_use]
    pub const fn silent() -> Self {
        Self {
            head: None,
            body_yaw: None,
            antennas: None,
        }
    }

    /// The head provenance.
    #[must_use]
    pub const fn head(self) -> Option<Isometry3<f64>> {
        self.head
    }

    /// The body-yaw provenance.
    #[must_use]
    pub const fn body_yaw(self) -> Option<f64> {
        self.body_yaw
    }

    /// The antenna provenance.
    #[must_use]
    pub const fn antennas(self) -> Option<[f64; 2]> {
        self.antennas
    }

    /// Set the head provenance.
    pub fn set_head(&mut self, anchor: Option<Isometry3<f64>>) {
        self.head = anchor;
    }

    /// Set the body-yaw provenance.
    pub fn set_body_yaw(&mut self, anchor: Option<f64>) {
        self.body_yaw = anchor;
    }

    /// Set the antenna provenance.
    pub fn set_antennas(&mut self, anchor: Option<[f64; 2]>) {
        self.antennas = anchor;
    }
}

impl OverlaySample {
    /// A sample that changes nothing, for a player that is between segments.
    #[must_use]
    pub fn silent() -> Self {
        Self {
            frame: DeltaFrame::zero(ChannelMask::empty()),
            weights: ChannelWeights::zero(),
            anchors: OverlayAnchors::silent(),
        }
    }

    /// Set the authored anchor for one channel.
    pub fn set_anchor(&mut self, channel: Channel, anchor: Option<JointTargets>) {
        match channel {
            Channel::Head => self
                .anchors
                .set_head(anchor.map(|targets| targets.head_pose_body)),
            Channel::BodyYaw => self
                .anchors
                .set_body_yaw(anchor.map(|targets| targets.body_yaw)),
            Channel::Antennas => self
                .anchors
                .set_antennas(anchor.map(|targets| targets.antennas)),
        }
    }
}

/// Fold `overlays` onto `base` and return the target to command.
///
/// Channels no overlay drives are copied from the base untouched.
#[must_use]
pub fn compose(base: JointTargets, overlays: &[OverlaySample]) -> JointTargets {
    let mut out = base;
    for posed in [true, false] {
        for sample in overlays {
            if let Some(delta) = sample.frame.head {
                let weight = sample.weights.get(Channel::Head);
                if weight > 0.0 && sample.anchors.head.is_some() == posed {
                    if let Some(anchor) = sample.anchors.head {
                        let target = anchor * delta;
                        out.head_pose_body = interpolate_pose(&out.head_pose_body, &target, weight);
                    } else {
                        out.head_pose_body *= scale_delta(&delta, weight);
                    }
                }
            }
            if let Some(delta) = sample.frame.body_yaw {
                let weight = sample.weights.get(Channel::BodyYaw);
                if weight > 0.0 && sample.anchors.body_yaw.is_some() == posed {
                    out.body_yaw = match sample.anchors.body_yaw {
                        Some(anchor) => lerp(out.body_yaw, anchor + delta, weight),
                        None => out.body_yaw + delta * weight,
                    };
                }
            }
            if let Some(delta) = sample.frame.antennas {
                let weight = sample.weights.get(Channel::Antennas);
                if weight > 0.0 && sample.anchors.antennas.is_some() == posed {
                    if let Some(anchor) = sample.anchors.antennas {
                        out.antennas[0] = lerp(out.antennas[0], anchor[0] + delta[0], weight);
                        out.antennas[1] = lerp(out.antennas[1], anchor[1] + delta[1], weight);
                    } else {
                        out.antennas[0] += delta[0] * weight;
                        out.antennas[1] += delta[1] * weight;
                    }
                }
            }
        }
    }
    out
}

/// Uncompose the overlays from `composed`, retaining `held` for posed channels.
///
/// What a caller needs it for: an overlay that stops riding a base takes its
/// whole weighted delta out of the composed stream in one period unless the base
/// absorbs it. Re-anchoring the base at this — the composed setpoint that was
/// last commanded, less what still rides it — makes the next composed setpoint
/// the same setpoint, and the offset then decays as a planned move under
/// whatever bounds the caller's own commands are held to.
///
/// A vacated unposed contribution on a posed channel leaves in one period;
/// unposed-only channels are unwound exactly.
#[must_use]
pub fn uncompose(
    composed: JointTargets,
    overlays: &[OverlaySample],
    held: &JointTargets,
) -> JointTargets {
    let mut out = composed;
    let posed = |channel: Channel| {
        overlays.iter().any(|sample| {
            sample.weights.get(channel) > 0.0
                && match channel {
                    Channel::Head => sample.anchors.head.is_some(),
                    Channel::BodyYaw => sample.anchors.body_yaw.is_some(),
                    Channel::Antennas => sample.anchors.antennas.is_some(),
                }
        })
    };
    if posed(Channel::Head) {
        out.head_pose_body = held.head_pose_body;
    }
    if posed(Channel::BodyYaw) {
        out.body_yaw = held.body_yaw;
    }
    if posed(Channel::Antennas) {
        out.antennas = held.antennas;
    }
    for sample in overlays.iter().rev() {
        if let Some(delta) = sample.frame.head {
            let weight = sample.weights.get(Channel::Head);
            if weight > 0.0 && !posed(Channel::Head) && sample.anchors.head.is_none() {
                out.head_pose_body *= scale_delta(&delta, weight).inverse();
            }
        }
        if let Some(delta) = sample.frame.body_yaw
            && !posed(Channel::BodyYaw)
            && sample.anchors.body_yaw.is_none()
        {
            out.body_yaw -= delta * sample.weights.get(Channel::BodyYaw);
        }
        if let Some(delta) = sample.frame.antennas {
            let weight = sample.weights.get(Channel::Antennas);
            if !posed(Channel::Antennas) && sample.anchors.antennas.is_none() {
                out.antennas[0] -= delta[0] * weight;
                out.antennas[1] -= delta[1] * weight;
            }
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
            anchors: OverlayAnchors::silent(),
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
            anchors: OverlayAnchors::silent(),
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
            anchors: OverlayAnchors::silent(),
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
            anchors: OverlayAnchors::silent(),
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
            anchors: OverlayAnchors::silent(),
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
                anchors: OverlayAnchors::silent(),
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
    fn posed_samples_precede_unposed_samples_in_either_row_order() {
        let base = moved_base();
        let anchor = JointTargets {
            head_pose_body: Isometry3::translation(0.02, 0.0, 0.18),
            ..base
        };
        let mut posed = head_yaw_sample(0.2, 1.0);
        posed.set_anchor(Channel::Head, Some(anchor));
        let ordinary = head_yaw_sample(0.4, 1.0);
        let first = compose(base, &[ordinary, posed]);
        let second = compose(base, &[posed, ordinary]);
        let expected =
            anchor.head_pose_body * posed.frame.head.unwrap() * ordinary.frame.head.unwrap();
        assert!(
            (first.head_pose_body.translation.vector - expected.translation.vector).norm() < 1e-12
        );
        assert!(
            (second.head_pose_body.translation.vector - expected.translation.vector).norm() < 1e-12
        );
        assert!(first.head_pose_body.rotation.angle_to(&expected.rotation) < 1e-12);
        assert!(second.head_pose_body.rotation.angle_to(&expected.rotation) < 1e-12);
    }

    #[test]
    fn partial_posed_and_unposed_antennas_are_order_independent() {
        let base = moved_base();
        let anchor = JointTargets {
            antennas: [0.7, -0.8],
            ..base
        };
        let mut posed = antennas_sample(0.1, -0.2, 0.5);
        posed.set_anchor(Channel::Antennas, Some(anchor));
        let ordinary = antennas_sample(0.2, 0.3, 0.5);
        let one = compose(base, &[posed, ordinary]);
        let two = compose(base, &[ordinary, posed]);
        assert_eq!(one.antennas, two.antennas);
    }

    #[test]
    fn half_weight_two_live_posed_bases_fold_from_the_current_output() {
        let base = moved_base();
        let mut first = head_yaw_sample(0.0, 0.5);
        first.set_anchor(
            Channel::Head,
            Some(JointTargets {
                head_pose_body: Isometry3::translation(0.02, 0.0, 0.19),
                ..base
            }),
        );
        let mut second = head_yaw_sample(0.0, 0.5);
        second.set_anchor(
            Channel::Head,
            Some(JointTargets {
                head_pose_body: Isometry3::translation(-0.02, 0.0, 0.18),
                ..base
            }),
        );
        let once = compose(base, &[first]);
        let expected = interpolate_pose(
            &once.head_pose_body,
            &(second.anchors.head.unwrap() * second.frame.head.unwrap()),
            0.5,
        );
        let twice = compose(base, &[first, second]);
        assert!(
            (twice.head_pose_body.translation.vector - expected.translation.vector).norm() < 1e-12
        );
        assert!(twice.head_pose_body.rotation.angle_to(&expected.rotation) < 1e-12);
    }

    #[test]
    fn two_posed_drivers_keep_later_row_order() {
        let base = moved_base();
        let mut first = antennas_sample(0.0, 0.0, 1.0);
        let mut second = antennas_sample(0.0, 0.0, 1.0);
        first.set_anchor(
            Channel::Antennas,
            Some(JointTargets {
                antennas: [0.1, 0.2],
                ..base
            }),
        );
        second.set_anchor(
            Channel::Antennas,
            Some(JointTargets {
                antennas: [0.7, 0.8],
                ..base
            }),
        );
        let later = compose(base, &[first, second]).antennas;
        let other = compose(base, &[second, first]).antennas;
        assert!((later[0] - 0.7).abs() < 1e-12 && (later[1] - 0.8).abs() < 1e-12);
        assert!((other[0] - 0.1).abs() < 1e-12 && (other[1] - 0.2).abs() < 1e-12);
    }

    #[test]
    fn partial_posed_interpolation_reaches_the_absolute_target_monotonically() {
        let base = moved_base();
        let anchor = JointTargets {
            head_pose_body: Isometry3::from_parts(
                Translation3::new(0.02, -0.01, 0.19),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3)),
            ),
            body_yaw: -0.2,
            antennas: [0.8, -0.6],
        };
        let delta = Isometry3::from_parts(
            Translation3::new(0.01, -0.02, 0.03),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.03, -0.02, 0.04)),
        );
        let mut sample = OverlaySample {
            frame: DeltaFrame {
                head: Some(delta),
                body_yaw: Some(0.1),
                antennas: Some([0.2, -0.1]),
            },
            weights: ChannelWeights::zero(),
            anchors: OverlayAnchors::silent(),
        };
        sample.set_anchor(Channel::Head, Some(anchor));
        sample.set_anchor(Channel::BodyYaw, Some(anchor));
        sample.set_anchor(Channel::Antennas, Some(anchor));
        let mut previous = f64::INFINITY;
        for weight in [0.0, 0.25, 0.5, 0.75, 1.0] {
            sample.weights = ChannelWeights::full();
            sample.weights.set(Channel::Head, weight);
            sample.weights.set(Channel::BodyYaw, weight);
            sample.weights.set(Channel::Antennas, weight);
            let out = compose(base, &[sample]);
            let target = anchor.head_pose_body * delta;
            let distance =
                (out.head_pose_body.translation.vector - target.translation.vector).norm();
            assert!(
                distance <= previous + 1e-12,
                "weight {weight}: {distance} > {previous}"
            );
            previous = distance;
            assert!(
                (out.body_yaw - (base.body_yaw + weight * (anchor.body_yaw + 0.1 - base.body_yaw)))
                    .abs()
                    < 1e-12
            );
            assert!(
                (out.antennas[0]
                    - (base.antennas[0] + weight * (anchor.antennas[0] + 0.2 - base.antennas[0])))
                    .abs()
                    < 1e-12
            );
        }
    }

    #[test]
    fn set_anchor_changes_only_the_selected_projection() {
        let target = JointTargets {
            head_pose_body: Isometry3::translation(0.11, -0.12, 0.13),
            body_yaw: 0.27,
            antennas: [0.31, -0.42],
        };
        let mut sample = OverlaySample::silent();
        sample.set_anchor(Channel::Head, Some(target));
        assert_eq!(sample.anchors.head, Some(target.head_pose_body));
        assert_eq!(sample.anchors.body_yaw, None);
        assert_eq!(sample.anchors.antennas, None);
        sample.set_anchor(Channel::BodyYaw, Some(target));
        assert_eq!(sample.anchors.body_yaw, Some(target.body_yaw));
        assert_eq!(sample.anchors.antennas, None);
        sample.set_anchor(Channel::Antennas, Some(target));
        assert_eq!(sample.anchors.antennas, Some(target.antennas));
    }

    #[test]
    fn a_silent_sample_changes_nothing() {
        let base = moved_base();
        assert_eq!(compose(base, &[OverlaySample::silent()]), base);
    }

    #[test]
    fn posed_channels_target_their_anchor_and_uncompose_keeps_them() {
        let base = moved_base();
        let anchor = JointTargets {
            head_pose_body: Isometry3::from_parts(
                Translation3::new(0.01, 0.02, 0.18),
                UnitQuaternion::from_scaled_axis(Vector3::z() * 0.2),
            ),
            body_yaw: 0.4,
            antennas: [0.2, -0.3],
        };
        let mut posed = OverlaySample {
            frame: DeltaFrame {
                head: Some(Isometry3::identity()),
                body_yaw: Some(0.1),
                antennas: Some([0.05, -0.05]),
            },
            weights: ChannelWeights::full(),
            anchors: OverlayAnchors {
                head: Some(anchor.head_pose_body),
                body_yaw: Some(anchor.body_yaw),
                antennas: Some(anchor.antennas),
            },
        };
        let full = compose(base, &[posed]);
        assert!(
            (full.head_pose_body.inverse() * anchor.head_pose_body)
                .translation
                .vector
                .norm()
                < 1e-12
        );
        assert!(
            (full.head_pose_body.inverse() * anchor.head_pose_body)
                .rotation
                .angle()
                < 1e-12
        );
        assert_eq!(full.body_yaw, anchor.body_yaw + 0.1);
        assert_eq!(full.antennas, [0.25, -0.35]);
        posed.weights.set(Channel::Head, 0.0);
        posed.weights.set(Channel::BodyYaw, 0.0);
        posed.weights.set(Channel::Antennas, 0.0);
        assert_eq!(compose(base, &[posed]), base);

        let mut mixed = posed;
        mixed.weights = ChannelWeights::full();
        mixed.set_anchor(Channel::Head, Some(anchor));
        mixed.set_anchor(Channel::BodyYaw, None);
        mixed.set_anchor(Channel::Antennas, None);
        let composed = compose(base, &[mixed]);
        let handed_back = uncompose(composed, &[mixed], &base);
        assert_eq!(handed_back.head_pose_body, base.head_pose_body);
        assert_eq!(handed_back.body_yaw, base.body_yaw);
        assert_eq!(handed_back.antennas, base.antennas);
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
            let found = uncompose(composed, overlays, &base);
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
        assert_eq!(uncompose(base, &[], &base), base);
    }

    #[test]
    fn posed_uncompose_holds_the_base_while_unrelated_content_vacates() {
        let base = moved_base();
        let anchor = JointTargets {
            head_pose_body: Isometry3::translation(0.01, 0.02, 0.19),
            ..base
        };
        for weight in [0.1, 0.5, 0.9, 1.0] {
            let mut posed = head_yaw_sample(0.2, weight);
            posed.set_anchor(Channel::Head, Some(anchor));
            let antenna = antennas_sample(0.2, -0.3, weight);
            let composed = compose(base, &[posed, antenna]);
            let all = uncompose(composed, &[posed, antenna], &base);
            assert!(
                (all.head_pose_body.translation.vector - base.head_pose_body.translation.vector)
                    .norm()
                    < 1e-12
                    && (all.antennas[0] - base.antennas[0]).abs() < 1e-12
                    && (all.antennas[1] - base.antennas[1]).abs() < 1e-12,
                "weight {weight}: {all:?}"
            );
            let antenna_vacated = uncompose(composed, &[posed], &base);
            assert_eq!(antenna_vacated.head_pose_body, base.head_pose_body);
            assert_eq!(antenna_vacated.antennas, composed.antennas);
            let recomposed = compose(antenna_vacated, &[posed]);
            assert_eq!(recomposed.head_pose_body, composed.head_pose_body);

            let head_overlay = head_yaw_sample(0.5, weight);
            let vacated = compose(base, &[posed, head_overlay]);
            let held = uncompose(vacated, &[posed], &base);
            assert_eq!(held.head_pose_body, base.head_pose_body);
            let recomposed = compose(held, &[posed]);
            let expected = compose(base, &[posed]);
            assert_eq!(recomposed.head_pose_body, expected.head_pose_body);
        }
    }

    #[test]
    fn posed_head_and_antenna_round_trips_return_the_held_base_at_partial_weights() {
        let base = moved_base();
        let anchor = JointTargets {
            head_pose_body: Isometry3::translation(0.01, 0.02, 0.19),
            antennas: [0.6, -0.7],
            ..base
        };
        for (channel, weight) in [(Channel::Head, 0.3), (Channel::Antennas, 0.75)] {
            let mut sample =
                head_yaw_sample(if channel == Channel::Head { 0.5 } else { 0.0 }, weight);
            if channel == Channel::Antennas {
                sample = antennas_sample(0.2, -0.3, weight);
            }
            sample.set_anchor(channel, Some(anchor));
            let composed = compose(base, &[sample]);
            let returned = uncompose(composed, &[sample], &base);
            assert!(
                (returned.head_pose_body.translation.vector
                    - base.head_pose_body.translation.vector)
                    .norm()
                    < 1e-12
            );
            assert!((returned.antennas[0] - base.antennas[0]).abs() < 1e-12);
            assert!((returned.antennas[1] - base.antennas[1]).abs() < 1e-12);
        }
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
