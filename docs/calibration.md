# PID and odometry calibration

Calibrate on a charged battery with the final wheel, mass, and payload. Record
raw data and change one variable at a time. Keep the robot on blocks until
wheel signs and stops are proven.

## Current starting values

Firmware:

- x4 quadrature counts/revolution: `1976.1`
- PID rate: `50 Hz`
- telemetry: `20 Hz`
- target ceiling: `120 RPM`
- gains per wheel: `Kp=0.80`, `Ki=0.30`, `Kd=0.05`
- RPM filter alpha: `0.30`
- minimum PWM: `45`; maximum PWM: `240`

ROS bridge:

- wheel radius: `0.050 m`
- center-to-wheel distance: `0.2921 m`
- maximum command scaling: `0.30 m/s`, `1.00 rad/s`

These are implementation values, not a guarantee of calibrated performance.

## Encoder and direction checks

1. Mark each wheel and rotate it exactly one output revolution by hand.
2. Record count change in both directions; repeat several times.
3. Confirm the x4 count and determine each encoder sign.
4. Command one lifted wheel at low positive RPM and confirm positive measured
   RPM. Fix direction constants or wiring before PID tuning.
5. Verify forward, left, and counter-clockwise chassis commands produce
   positive ROS `vx`, `vy`, and `wz`.

## Velocity-loop tuning

Tune each lifted wheel first, then repeat loaded on the floor:

1. Set I and D to zero. Apply small RPM steps and raise P until response is
   prompt without sustained oscillation.
2. Add I slowly to remove steady-state error. Confirm anti-windup recovers
   after a blocked or saturated wheel.
3. Add D only if measured data shows it improves damping; encoder
   quantization can make D noisy.
4. Characterize breakaway PWM in both directions rather than assuming `45`.
5. Test several RPMs and battery voltages. Preserve control headroom below
   no-load speed.

Log target RPM, measured/raw RPM, PWM, encoder count, battery voltage, load,
and timestamp. Commit calibrated constants only with test conditions.

## Geometry calibration

For wheel radius, command a long straight run and compare odometry distance
with a tape-measured distance. A first correction is:

```text
new_radius = old_radius * measured_distance / odom_distance
```

Repeat in both directions. For center-to-wheel distance, perform slow
multi-turn rotations and compare external yaw with odometry yaw:

```text
new_distance = old_distance * odom_yaw / measured_yaw
```

Validate lateral translation separately; omni-wheel slip can differ strongly
between forward and lateral travel.

## Covariance and acceptance

Do not tune covariance to make a plot look stable. Run repeated straight,
lateral, rotational, and closed-loop paths; calculate endpoint error and
variance. Wheel-only odometry should receive realistic uncertainty and will
still drift. Consider IMU fusion as a future improvement.

Acceptance notes should include firmware commit, geometry, surface, payload,
battery voltage, repetitions, mean error, spread, and rosbag location.
