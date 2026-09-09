# KiwiDrive: Three-Wheel Holonomic Robot

[![ROS 2 Lyrical CI](https://github.com/jbriceno225/3-Axis-Holonomic-Drive-Robot/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/jbriceno225/3-Axis-Holonomic-Drive-Robot/actions/workflows/ros2-ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

KiwiDrive is a portfolio-scale mobile robotics platform built around three
120-degree omni wheels, an ESP32 motor controller, a Raspberry Pi 5, and an
LD19 2D LiDAR. The repository brings the mechanical design, Rev A electronics,
embedded velocity control, and a ROS 2 Lyrical integration stack into one
reproducible project.

> **Current state:** manual drive, firmware control logic, CAD/PCB source, ROS
> description, serial protocol, kinematics, odometry, launch/configuration, and
> Python unit tests are present. Full physical-system validation is still in
> progress. In particular, Nav2 mapping, localization, and autonomous frontier
> exploration must not be treated as demonstrated capabilities yet.

The checked-in PDF command guide is a legacy reference. Its unstamped Twist
examples are outdated; the current bridge consumes
`geometry_msgs/msg/TwistStamped`. Use [the serial protocol](docs/serial-protocol.md)
and [mapping/navigation guide](docs/mapping-navigation.md) instead.

## Rev A status

Rev A is a module-carrier/control PCB and assembled-system integration stage:

- KiCad source, Gerbers, drill files, and a purchasing BOM are checked in.
- The board carries an ESP32 DevKit, three TB6612FNG driver modules, a 5 V
  converter, motor/encoder connections, test points, and battery input.
- Firmware implements Bluepad32 manual control, 50 Hz per-wheel velocity PID,
  quadrature encoder capture, a 300 ms autonomous-command watchdog, and
  line-oriented serial telemetry.
- ROS code converts stamped body commands to normalized serial commands,
  reconstructs wheel odometry, publishes `odom -> base_link`, and provides
  conservative SLAM/Nav2/exploration configuration.
- Firmware dependency versions, LiDAR pose, footprint, wheel geometry, PID
  gains, odometry signs, and covariance still require system-level
  confirmation. Autonomous exploration has not been validated on hardware.

## Architecture

```text
Nav2 / operator
      |
      v  TwistStamped
velocity smoother -> collision monitor -> kiwi_serial_bridge
                                             |
                                  USB serial V,x,y,rotation
                                             |
                                             v
                     ESP32 -> wheel PID -> TB6612 modules -> motors
                       ^                                  |
                       +----------- encoders ------------+
                       |
              ODOM,rpm1,rpm2,rpm3,counts...
                       |
                       v
                 /odom + odom -> base_link

LD19 /scan -> SLAM Toolbox -> map -> odom -> Nav2
URDF/robot_state_publisher -> base_link -> base_laser
```

See [Architecture and ROS](docs/architecture-and-ros.md) for nodes, topics,
TF ownership, and coordinate conventions.

## Features and evidence

| Area | Repository evidence | Status |
| --- | --- | --- |
| Mechanical | Fusion 360, STEP, and printable STL files | Design artifacts available |
| Electronics | Rev A KiCad source and fabrication outputs | Fabrication package available; validate each build |
| Manual control | Bluepad32 gamepad path and latched software stop | Implemented in firmware |
| Wheel control | Encoder feedback, feed-forward, PID, anti-windup | Implemented; gains need load testing |
| ROS bridge | Stamped velocity input, watchdog, serial parsing, odometry/TF | Implemented; bench/robot validation pending |
| Robot model | Xacro, meshes, sensor frame, RViz launch | Static/config checks available |
| Mapping/Nav2 | SLAM Toolbox and holonomic Nav2 configuration | Configured, not yet demonstrated |
| Frontier exploration | `explore_lite` integration and safety gate | Experimental and unvalidated |

## Hardware and software

Expected hardware includes a Raspberry Pi 5, ESP32 DevKit V1, three geared DC
motors with quadrature encoders, three omni wheels, three TB6612FNG driver
modules, LD19 LiDAR, Rev A PCB, regulated power, frame, and printed parts.
Confirm voltage/current ratings and add an external, power-cutting emergency
stop before floor testing.

The primary software target is Ubuntu 26.04 with ROS 2 Lyrical. Firmware uses
the Arduino ESP32 core and Bluepad32; exact tested versions remain owner
confirmation items. See [hardware](docs/hardware.md) and
[firmware setup](ESP32_Code/README.md).

## Quick start

On a supported Ubuntu/ROS host:

```bash
git clone --recurse-submodules \
  https://github.com/jbriceno225/3-Axis-Holonomic-Drive-Robot.git \
  ~/KiwiDrive
cd ~/KiwiDrive
bash scripts/bootstrap_ros2.sh
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

Static visualization (mock hardware only):

```bash
ros2 launch kiwi_drive_description display.launch.py
```

Physical hardware, after configuring persistent device names:

```bash
# First identify which temporary port belongs to each device.
ls -l /dev/ttyUSB*
# Replace these arguments if detection assigns the ports differently.
bash scripts/install_udev_rules.sh /dev/ttyUSB0 /dev/ttyUSB1
ros2 launch kiwi_bringup hardware.launch.py
```

Do not start autonomy as the first hardware test. Follow the staged validation
below.

## Safe staged validation

1. Inspect power polarity, continuity, wiring, wheel clearance, and mechanical
   fasteners with traction power disconnected.
2. Flash firmware; verify `PING`, mode switching, and encoder signs without
   enabling autonomous movement.
3. Lift the wheels. Test each axis at low command values and verify the 300 ms
   ESP32 timeout and ROS-side 250 ms timeout.
4. Confirm `/scan`, `/odom`, TF ownership, geometry, and REP-103 signs.
5. Hand-drive a small map before enabling Nav2.
6. Test short forward, lateral, and rotational goals in a clear bounded area.
7. Attempt reduced-speed exploration only after every earlier gate passes,
   with a spotter and physical power cutoff.

Use the command-by-command [validation checklist](docs/mapping-navigation.md#staged-hardware-validation).
Software stops and Ctrl-C are not substitutes for a physical emergency stop.

## Repository structure

```text
CAD/                 mechanical source, interchange, and print files
PCB/                 Rev A KiCad project and fabrication outputs
ESP32_Code/          current ESP32/Bluepad32 firmware sketch
docs/                system, calibration, operation, and development guides
patches/             first-party patch overlays for pinned vendor submodules
scripts/             bootstrap, patch, and udev helpers
src/kiwi_bringup/    physical launch and autonomy configuration
src/kiwi_description/ robot model plus clearly labeled generated diagnostics
src/kiwi_serial_bridge/ serial, kinematics, odometry, and ROS node
src/*vendor*/        third-party git submodules; retain upstream licensing
```

## Demo and media

No fabricated screenshots or performance results are included. To add a demo:

1. Record the date, commit, hardware revision, battery voltage, test area, and
   exact launch command.
2. Capture the robot and safety setup in-frame; avoid presenting simulation or
   lifted-wheel tests as floor-navigation results.
3. Save small images under `docs/media/`; link large videos from a durable
   host. Add captions describing what was actually validated.
4. For mapping/navigation evidence, include an RViz view and a rosbag or topic
   summary for `/scan`, `/odom`, `/tf`, `/map`, and the velocity pipeline.

Suggested placeholders once real evidence exists:

- `docs/media/rev-a-overview.jpg` — assembled Rev A hardware
- `docs/media/manual-holonomic-drive.mp4` — manual translation/rotation
- `docs/media/nav2-validation.mp4` — only after staged Nav2 validation passes

## Known limitations

- No recorded end-to-end Nav2 or autonomous-exploration pass is committed.
- Wheel radius (`0.050 m`) and center-to-wheel distance (`0.2921 m`) are
  current measured/configured values but need loaded odometry calibration.
- LiDAR transform and costmap footprint are provisional until remeasured on
  the final assembly.
- Wheel-only odometry drifts and has no IMU fusion.
- The Rev A electrical design relies on carrier modules and lacks the planned
  Rev B protection, switching, diagnostics, and integrated driver circuitry.
- The legacy PDF contains an outdated velocity-message contract.

## Rev B roadmap

- Add a physical master switch and accessible hard power cutoff.
- Add branch fusing for battery, logic, LiDAR/Pi, and motor loads.
- Add bulk/local decoupling and the required pull-ups/pull-downs.
- Add power, fault, mode, and driver-status indicators.
- Integrate appropriately rated motor-driver circuitry instead of carrier
  modules, with thermal/current margin and test access.
- Confirm connector keying, grounding, protection, current return paths,
  design-rule checks, and fabrication documentation before release.
- Lock calibrated geometry, PID, covariance, firmware toolchain versions, and
  repeatable test evidence.

## Documentation

- [Architecture and ROS](docs/architecture-and-ros.md)
- [Hardware and revisions](docs/hardware.md)
- [Serial protocol](docs/serial-protocol.md)
- [PID and odometry calibration](docs/calibration.md)
- [Mapping and navigation](docs/mapping-navigation.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Development and contributing](docs/development.md)
- [Security and safety reporting](SECURITY.md)

## License

First-party project material is licensed under the [MIT License](LICENSE).
Third-party submodules and imported/generated assets may carry their own
licenses and headers; those terms remain in force.
