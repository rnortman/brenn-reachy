# Frames — the head, the body, and the audio device's array

What this document is: the frames the motion code works in, and the convention
of the audio device's direction-of-arrival readings, stated once. The
implementation is `reachy_kin::yaw`, `reachy_kin::envelope` and
`reachy_kin::mic`.

## Frames

- Right-handed, z up, x forward (the gaze at neutral), so +y is the robot's
  left and positive yaw turns the head to its left.
- The solvers take the head pose relative to the body.
- Body yaw is the yaw servo's absolute angle.
- `body_to_world(pose_body, body_yaw) = Rz(body_yaw) · pose_body`.
- Head-relative yaw is the ZYX yaw of the body-frame head rotation, capped at
  ±55°. Body yaw is capped at ±160°.
- The yaw sign has not been confirmed on the unit. Confirm it with one commanded
  pose of positive head yaw (the head must turn to *its* left) before any
  bearing below is trusted.

## Where the head attitude comes from

- The control process's pose solver publishes one `PoseEstimate` per 20 ms
  sample: head position and orientation in the body frame, the measured joints
  (body yaw among them), a `valid` flag, and `time_of_validity`, the sample's
  own read time on the realtime clock.
- The online composition sends each one to the voice host on loopback port
  7411.

## The audio device's azimuths

- Four beams: `[0]` and `[1]` are focused (slow), `[2]` is free-running, and
  `[3]` is auto-select, a copy of one of the other three.
- An azimuth `α ∈ [0, π]` rad is the angle between the source direction and the
  **array axis**. It is a cone about that axis: broadside is `π/2`, front/back
  and up/down fold together, and no elevation is reported.
- NaN is lawful on 0, 1 and 3 when nothing is tracked.
- The device polls at 10 Hz and forwards readings to the host only while its
  voice-activity segment is open.

## The mount

- Four microphones in line on top of the head. The axis is perpendicular to the
  gaze: the head's +y.
- **The 0° end is the robot's left.** Measured on the unit: a talker straight
  ahead read ≈ 90°, one to the left 23–29°, one to the right 128–158°.
- An earlier run read a talker placed straight ahead at 103–107°. Whether that
  was placement or an offset of the axis is not established.

## The model

- `cos α = a · d`, where `a` is the array axis in the world frame (unit, toward
  the 0° end) and `d = (cos φ cos θ, cos φ sin θ, sin φ)` for a talker at world
  bearing θ (from world x, positive left) and elevation φ.
- Level and unyawed, `a = (0, 1, 0)`, so `cos α = cos φ sin θ`.
- Pitch leaves α unchanged. Yaw and roll move `a` and change it.
- The general solution, for any head attitude, is
  `reachy_kin::mic::bearing_from_azimuth`. Its two solutions are symmetric about
  the axis (the front/back fold); it takes the front one; a reading closer to
  end-fire than any source at φ can be is refused, and the edge of the cone on
  that side is reported.

## Elevation

- φ is an assumption, not a reading: a standing visitor's mouth above a
  table-top head at a metre or less.
- 20° is the working figure, the one the launcher passes. The caller supplies it.
- φ is also the pitch a look commands: the head pitches nose-up by φ, the
  elevation the bearing was solved under.
- The launcher carries it inside [0°, 30°], level up to 5° under the head's
  35° cone limit, the range every look is checked against the envelope in.
  The voice host refuses to start on a `--gaze-elevation-deg` outside it.
