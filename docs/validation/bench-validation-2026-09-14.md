# Raised-wheel drivetrain validation — 2026-09-14

**System:** Raspberry Pi 5, ROS 2 Lyrical, ESP32 Kiwi firmware, `feature/nav2-bringup`.

**Safety:** All tests were conducted with the three wheels raised. Each command was published for 5 seconds at 20 Hz, then an explicit zero `TwistStamped` was published. Hardware bringup was stopped after testing.

## Configuration under test

- Bridge SI-to-normalized scaling: linear `0.6283 m/s`, angular `1.7208 rad/s`
- Wheel radius: `0.050 m`; center-to-wheel distance: `0.2921 m`
- Firmware: `MAX_TARGET_RPM=120`, `MIN_PWM=0`, `TRANSLATION_SCALE=1.00`, `ROTATION_SCALE=0.80`
- PID: `Kp=0.80`, `Ki=0.30`, `Kd=0.05`

## Lateral translation

### ROS +Y (`linear.y=+0.20 m/s`)

- Normalized ESP32 command: `x=+0.3183`, `y=0`, `rotation=0`
- Expected wheel RPM `[M1, M2, M3]`: `[-19.1, +38.2, -19.1]`
- Measured steady samples: approximately `[-16.1..-17.1, +21.6..+28.7, -15.4..-17.0] RPM`
- Encoder counts changed from approximately `[129053, 7714, -137788]` to `[129591, 6806, -137252]` during the commanded interval.
- Odom delta: `dx=+0.0927 m`, `dy=+0.7824 m`, `dyaw=-12.0 deg`
- Result: **fail**. Direction was primarily +Y, but the wheel-speed ratio was about `-0.66 : +1 : -0.66` rather than `-0.5 : +1 : -0.5`, producing unacceptable yaw drift.

### ROS -Y (`linear.y=-0.20 m/s`)

- Normalized ESP32 command: `x=-0.3183`, `y=0`, `rotation=0`
- Expected wheel RPM `[M1, M2, M3]`: `[+19.1, -38.2, +19.1]`
- Measured steady samples: approximately `[+16.1..+17.2, -34.4..-35.6, +15.9..+17.0] RPM`
- Odom delta: `dx=-0.3128 m`, `dy=-0.8992 m`, `dyaw=-3.55 deg`
- Result: **fail**. The ratio was much closer to target than +Y, but still has yaw and X drift beyond the acceptance gate.

## Isolated motor-2 speed sweep

Motor 2 was isolated kinematically by commanding coordinated ROS lateral and angular velocity. Approximate steady Motor-2 response:

| Target RPM | +RPM measured | -RPM measured |
| ---: | ---: | ---: |
| 5 | 1–2 | 1–3 |
| 10 | 6–7 | 6–7 |
| 15 | 10–11 | 11–12 |
| 20 | 15–16 | 15–16 |
| 30 | 24–25 | 24–25 |
| 40 | 33–34 | 33–35 |
| 60 | 51–53 | 51–52 |

Result: the response is approximately symmetric by direction, has a substantial low-speed deficit, and improves with RPM. This does **not** support changing bridge scaling.

## Equal-target rotation and mixed-polarity controls

### Pure CCW rotation

- ROS command: `angular.z=+0.684735 rad/s`
- Normalized command: `rotation=+0.3979`; firmware rotational contribution `+0.3183`
- Expected wheel RPM: `[+38.2, +38.2, +38.2]`
- Measured steady samples: approximately `[+35.2..+36.0, +35.0..+35.5, +34.9..+35.9] RPM`
- Result: **pass for wheel-ratio symmetry**. All wheels track each other closely at the same target.

### Mixed-polarity equal-speed controls

- Targets `[-38.2, +38.2, -38.2] RPM`: measured approximately `[-35..-36, +34..+36, -35..-36] RPM`.
- Targets `[+38.2, -38.2, +38.2] RPM`: measured approximately `[+34..+36, -34..-36, +34..+36] RPM`.
- Result: **pass for equal-target symmetry**. There is no evidence of an individual M2, encoder, or polarity-specific fault at equal RPM.

## Conclusion and next gate

The remaining fault is **mixed-speed low-RPM tracking**, not ROS-to-ESP32 scaling, wheel-sign configuration, or a single-wheel direction error. The +Y lateral command fails because the required 2:1 wheel-speed relationship is not maintained at different target magnitudes.

Do not proceed to floor calibration, SLAM mapping, or Nav2 motion yet. Next work must add non-disruptive target-RPM, measured-RPM, and PWM telemetry to the firmware, flash it with a reproducible toolchain, and use the same raised-wheel sweep to decide between feed-forward/static-friction compensation and PID changes. Change one control variable at a time.
