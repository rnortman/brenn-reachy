//! The linkage's numeric constants, transcribed once so nothing downstream has
//! to parse a data file at runtime.
//!
//! Provenance for everything in this module is the Apache-2.0 `reachy_mini`
//! distribution: the link lengths, crank transforms and platform anchors come
//! from its `assets/kinematics_data.json`, and the stow pose from the sleep
//! head pose in its daemon backend. A copy of the JSON is vendored under
//! `tests/fixtures/` and [`tests`] parses it and compares the numbers taken
//! from it — the two link lengths, the head offset, the six crank transforms
//! and the six anchors — against it, so a transcription slip in those cannot
//! survive a test run.
//!
//! The crank travel windows come from the vendor's MJCF model instead, which
//! states them as exact whole degrees. They are deliberately **not** taken from
//! the JSON's per-motor `limits` field: that field is written as a hard-coded
//! ±π by the generator that produces the file and describes nothing about the
//! hardware. Having no fixture, the windows are pinned by the travel sweep in
//! [`crate::ik`], which asserts where each of them binds; the stow pose is
//! pinned only by the crank angles it produces there.

use crate::geometry::BranchSign;

/// Crank (motor arm) length, metres.
pub const CRANK_LEN: f64 = 0.04000000000000001;

/// Rod length, metres.
pub const ROD_LEN: f64 = 0.08499999999999995;

/// Height of the head frame origin above the base frame at the neutral pose,
/// metres.
pub const HEAD_Z_OFFSET: f64 = 0.177;

/// Base frame → crank frame, per leg, as the top three rows of a 4×4
/// homogeneous transform (row-major, translation in the fourth column).
///
/// The direction is base → crank, i.e. the inverse of the crank frame's pose in
/// the base frame. Reading these the other way round produces a plausible but
/// wrong hexagon of crank axes, so the direction is asserted in [`tests`] by
/// reconstructing the crank origins and checking they land on one circle at one
/// height.
///
/// The frames already include the 7 mm slide along each crank's own axis that
/// lets the crank be modelled as a flat 0.040 m arm in the crank frame's local
/// xy plane, rotating about that frame's local z. Crank frames read from the
/// vendor's URDF do not have that slide and must never be mixed in here.
pub const BASE_TO_CRANK: [[[f64; 4]; 3]; 6] = [
    [
        [
            0.8660247915798898,
            -0.5000010603626028,
            -2.298079077119539e-06,
            -0.009999848080267933,
        ],
        [
            4.490195936008854e-06,
            3.1810770986818273e-06,
            0.999999999984859,
            -0.07663346037245178,
        ],
        [
            -0.500001060347722,
            -0.8660247915770963,
            4.999994360718464e-06,
            0.03666015757925319,
        ],
    ],
    [
        [
            -0.8660211183436269,
            0.5000074225224785,
            2.298069723064582e-06,
            -0.01000055227585102,
        ],
        [
            -4.490219645842903e-06,
            -3.181063409649239e-06,
            -0.999999999984859,
            0.07663346037219607,
        ],
        [
            -0.5000074225075973,
            -0.8660211183408337,
            5.00001124330122e-06,
            0.03666008712637943,
        ],
    ],
    [
        [
            6.326794896519466e-06,
            0.9999999999799852,
            -7.0550646912150425e-12,
            -0.009999884140839245,
        ],
        [
            -1.0196153102346142e-06,
            1.3505961633338446e-11,
            0.9999999999994795,
            -0.07663346037438698,
        ],
        [
            0.9999999999794655,
            -6.326794896940104e-06,
            1.0196153098685706e-06,
            0.036660683387545835,
        ],
    ],
    [
        [
            -3.673205069955933e-06,
            -0.9999999999932537,
            -6.767968877969483e-14,
            -0.010000000000897517,
        ],
        [
            1.0196153102837198e-06,
            -3.6775764393585005e-12,
            -0.9999999999994795,
            0.0766334603742898,
        ],
        [
            0.9999999999927336,
            -3.673205070385213e-06,
            1.0196153102903487e-06,
            0.03666065685180194,
        ],
    ],
    [
        [
            -0.8660284647694133,
            -0.4999946981757419,
            2.298079429767357e-06,
            -0.010000231529504576,
        ],
        [
            4.490172883391843e-06,
            -3.1811099293773187e-06,
            0.9999999999848591,
            -0.07663346037246624,
        ],
        [
            -0.4999946981608617,
            0.8660284647666201,
            4.999994384073154e-06,
            0.03666016059492482,
        ],
    ],
    [
        [
            0.8660247915798897,
            0.5000010603626025,
            -2.298069644866714e-06,
            -0.009999527331574583,
        ],
        [
            -4.490196220318687e-06,
            3.1810964558725514e-06,
            -0.9999999999848591,
            0.07663346037272492,
        ],
        [
            -0.500001060347722,
            0.8660247915770967,
            5.000011266610794e-06,
            0.036660231042625266,
        ],
    ],
];

/// Rod anchor on the moving platform, per leg, in the head frame.
///
/// All six are coplanar at z ≈ 0 on a circle of radius 0.030 m, so the head
/// frame origin sits in the anchor plane.
pub const ANCHOR_HEAD: [[f64; 3]; 6] = [
    [
        0.020648178337122566,
        0.021763723638894568,
        1.0345743467476964e-07,
    ],
    [
        0.00852381571767217,
        0.028763668526131346,
        1.183437210727778e-07,
    ],
    [
        -0.029172011376922807,
        0.0069999429399361995,
        4.0290270064691214e-08,
    ],
    [
        -0.029172040355214434,
        -0.0069999960097160766,
        -3.1608172912367394e-08,
    ],
    [
        0.008523809101930114,
        -0.028763713010385224,
        -1.4344916837716326e-07,
    ],
    [
        0.020648186722822436,
        -0.02176369606185343,
        -8.957920105689965e-08,
    ],
];

/// Which root of the loop closure each leg occupies, alternating by leg parity
/// because the two legs of a pair are mirror images about their shared plane.
///
/// These signs are not read off the JSON's per-motor `solution` field — that
/// field's 0/1 encoding says nothing about which of *our* two algebraic roots it
/// names. They are pinned instead by the crank travel windows: at the neutral
/// pose exactly one root per leg lies inside that leg's own window, and
/// [`crate::ik::tests`] asserts these signs select it.
pub const BRANCH_SIGNS: [BranchSign; 6] = [
    BranchSign::Minus,
    BranchSign::Plus,
    BranchSign::Minus,
    BranchSign::Plus,
    BranchSign::Minus,
    BranchSign::Plus,
];

/// Per-leg crank travel window, `(min, max)` in **degrees**.
///
/// Asymmetric on purpose: legs 2 and 5 have 22° more travel on their far side
/// than the other four. At the bottom of vertical travel legs 1, 3, 4 and 6 all
/// reach ∓48° while 2 and 5 still have that 22° in hand, so the stop that holds
/// the linkage off its lower singular configuration rests on four legs. A
/// symmetric envelope taken from legs 2 and 5 would drive straight through it.
pub const CRANK_WINDOWS_DEG: [(f64, f64); 6] = [
    (-48.0, 80.0),
    (-80.0, 70.0),
    (-48.0, 80.0),
    (-80.0, 48.0),
    (-70.0, 80.0),
    (-80.0, 48.0),
];

/// Stow head translation in the base frame, metres: the vendor's sleep head
/// pose translation with [`HEAD_Z_OFFSET`] added to reach the base frame.
pub const STOW_TRANSLATION: [f64; 3] = [-0.021, 0.001, HEAD_Z_OFFSET - 0.044];

/// Stow head pitch, radians (+24.387°).
///
/// The vendor states the sleep orientation as a rotation matrix rounded to three
/// decimals, which is not orthonormal; this is the pitch its first column
/// implies, and the stow pose is rebuilt as an exact rotation about the head y
/// axis. The reconstruction moves the resulting crank angles by at most 0.3°
/// against the rounded matrix.
pub const STOW_PITCH: f64 = 0.425_634_609_124_168_34;

/// Head translation of the tight resting configuration, base frame, metres.
///
/// A recorded observation, not a derivation: this is the configuration the
/// vendor's simulated backends start from, and it sits 0.141 mm from a singular
/// configuration of the linkage — a twentieth of the clearance floor commands are
/// held to. It is the configuration the clearance baseline exists for, so it is
/// baked here once rather than retyped by each test that needs a rest tighter
/// than the floor.
///
/// The same configuration is on record twice, as this pose and as
/// [`REST_CRANK_ANGLES_DEG`], and **the two records disagree**: run through the
/// solvers they differ by 3.689 µm of translation and 0.306° of pitch, and the
/// clearance they imply differs by a third — 0.141 mm from this pose against
/// 0.182 mm from the angles. Both were written to two decimals, so the pitch gap
/// is thirty times either record's own precision. This pose is the record every
/// caller uses, and the tighter of the two. The gaps are pinned as goldens by
/// the tests that cross-check them.
pub const REST_TRANSLATION: [f64; 3] = [-0.015_17, 0.001_03, 0.126_57];

/// Head pitch of the tight resting configuration, **degrees**, about the head y
/// axis.
pub const REST_PITCH_DEG: f64 = 30.84;

/// Crank angles of the tight resting configuration, **degrees**, in servo order
/// 1..=6.
pub const REST_CRANK_ANGLES_DEG: [f64; 6] = [-56.43, 72.33, -13.97, 11.78, -70.84, 57.48];

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    /// Every baked number, against the vendored copy of the file it came from.
    /// Exact equality: these are transcriptions, not derivations.
    #[test]
    fn baked_constants_match_the_vendored_source() {
        // Compiled in rather than read at run time. The path is relative to
        // this source file, so both build lanes reach the same bytes without a
        // manifest directory or a runfiles tree in either of them.
        let text = include_str!("../tests/fixtures/kinematics_data.json");
        let json: Value = serde_json::from_str(text).expect("fixture parses");

        assert_eq!(json["motor_arm_length"].as_f64().unwrap(), CRANK_LEN);
        assert_eq!(json["rod_length"].as_f64().unwrap(), ROD_LEN);
        assert_eq!(json["head_z_offset"].as_f64().unwrap(), HEAD_Z_OFFSET);

        let motors = json["motors"].as_array().expect("motors array");
        assert_eq!(motors.len(), 6);
        for (leg, motor) in motors.iter().enumerate() {
            let m = motor["T_motor_world"].as_array().unwrap();
            for row in 0..3 {
                let cols = m[row].as_array().unwrap();
                for col in 0..4 {
                    assert_eq!(
                        cols[col].as_f64().unwrap(),
                        BASE_TO_CRANK[leg][row][col],
                        "leg {} transform [{}][{}]",
                        leg + 1,
                        row,
                        col
                    );
                }
            }
            let anchor = motor["branch_position"].as_array().unwrap();
            for axis in 0..3 {
                assert_eq!(
                    anchor[axis].as_f64().unwrap(),
                    ANCHOR_HEAD[leg][axis],
                    "leg {} anchor axis {}",
                    leg + 1,
                    axis
                );
            }
        }
    }

    /// The branch signs alternate strictly by leg parity, which the pair
    /// mirror symmetry forces. Which parity takes which sign is settled in
    /// `ik::tests`, against the travel windows.
    #[test]
    fn branch_signs_alternate_by_parity() {
        for leg in 0..6 {
            assert_eq!(BRANCH_SIGNS[leg], BRANCH_SIGNS[leg % 2]);
        }
        assert_ne!(BRANCH_SIGNS[0], BRANCH_SIGNS[1]);
    }

    /// The transforms are base → crank, so inverting one puts the crank origin
    /// in the base frame. All six then land on a single circle of radius
    /// 38.0 mm at a single height, which the opposite reading does not produce.
    #[test]
    fn transforms_are_base_to_crank() {
        for (leg, t) in BASE_TO_CRANK.iter().enumerate() {
            // origin = -Rᵀ·translation
            let mut origin = [0.0f64; 3];
            for axis in 0..3 {
                origin[axis] =
                    -(t[0][axis] * t[0][3] + t[1][axis] * t[1][3] + t[2][axis] * t[2][3]);
            }
            let radius = origin[0].hypot(origin[1]);
            assert!(
                (radius - 0.038).abs() < 1e-6,
                "leg {} crank radius {radius}",
                leg + 1
            );
            assert!(
                (origin[2] - 0.076_633).abs() < 1e-6,
                "leg {} crank height {}",
                leg + 1,
                origin[2]
            );
        }
    }

    /// The rotation blocks are orthonormal to machine precision *and* proper
    /// rotations, which is what lets [`crate::geometry`] take them as rotations
    /// without renormalising. Orthonormality alone would admit a reflection,
    /// and a reflection converts to a quaternion for some other rotation
    /// entirely — a mirrored linkage that solves and is wrong.
    #[test]
    fn rotation_blocks_are_proper_rotations() {
        for (leg, t) in BASE_TO_CRANK.iter().enumerate() {
            for i in 0..3 {
                for j in 0..3 {
                    let dot: f64 = (0..3).map(|k| t[i][k] * t[j][k]).sum();
                    let expected = if i == j { 1.0 } else { 0.0 };
                    assert!(
                        (dot - expected).abs() < 1e-12,
                        "leg {} rows {i},{j} dot {dot}",
                        leg + 1
                    );
                }
            }
            let det = t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1])
                - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0])
                + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0]);
            assert!((det - 1.0).abs() < 1e-12, "leg {} det {det}", leg + 1);
        }
    }

    /// The anchors sit on one circle of radius 30.0 mm in the anchor plane.
    #[test]
    fn anchors_are_coplanar_at_platform_radius() {
        for (leg, a) in ANCHOR_HEAD.iter().enumerate() {
            assert!((a[0].hypot(a[1]) - 0.030).abs() < 1e-6, "leg {}", leg + 1);
            assert!(a[2].abs() < 1e-6, "leg {}", leg + 1);
        }
    }
}
