# Development and contributing

Contributions should preserve the distinction between implemented,
bench-tested, and physically validated behavior. Never claim navigation,
mapping accuracy, safety, or performance without reproducible evidence.

## Set up

Use Ubuntu 26.04 with ROS 2 Lyrical:

```bash
git clone --recurse-submodules \
  https://github.com/jbriceno225/3-Axis-Holonomic-Drive-Robot.git \
  ~/KiwiDrive
cd ~/KiwiDrive
bash scripts/bootstrap_ros2.sh
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

The bootstrap installs ROS dependencies, applies first-party patch overlays to
pinned vendor submodules, and builds the workspace. Do not edit third-party
submodule code directly unless the change is intentionally captured as a
patch and its upstream license/header remains intact.

GitHub Actions uses the official
`ros:lyrical-ros-base-resolute` container. It checks an amd64 clean software
build, Python syntax, package lint, and unit tests; it cannot compile the
Arduino sketch, inspect electronics/CAD, access USB devices, or validate
physical motion.

## Before a pull request

```bash
(cd src/kiwi_serial_bridge && python3 -m pytest test)
python3 -m compileall -q \
  src/kiwi_serial_bridge/kiwi_serial_bridge \
  src/kiwi_bringup/launch \
  src/kiwi_description/kiwi_drive_description/launch
colcon build --symlink-install --packages-up-to kiwi_bringup
colcon test --packages-select \
  kiwi_serial_bridge kiwi_drive_description kiwi_bringup
colcon test-result --verbose
git diff --check
```

Also run the KiCad electrical/design-rule checks for PCB changes and inspect
exported CAD/meshes for mechanical changes. Generated fabrication or robot
description artifacts must identify their source tool and should be reviewed,
not treated as hand-authored truth.

## Change expectations

- Add/update unit tests for protocol, kinematics, and odometry logic.
- Keep ROS topic types, units, frame names, and TF ownership explicit.
- Update firmware, protocol docs, parser, tests, and configs atomically when
  changing the serial contract.
- Include calibration conditions and raw evidence for tuned constants.
- Avoid binary-only changes when an editable source format exists.
- Do not commit build trees, logs, `__pycache__`, serial-port identifiers, or
  secrets.
- Leave upstream copyright/license headers unchanged.

## Pull requests

Use the repository template. Keep changes focused and state:

- what changed and why;
- hardware/ROS versions affected;
- checks performed and checks not performed;
- safety impact and rollback;
- whether results are static, simulated, lifted-wheel, or floor-tested.

For safety or security concerns, use [SECURITY.md](../SECURITY.md) instead of
publishing sensitive exploit details.
