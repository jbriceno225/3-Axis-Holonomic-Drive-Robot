# Troubleshooting

Always stop motion and disconnect traction power before changing wiring.

## ESP32 device is missing

```bash
ls -l /dev/kiwi_esp32 /dev/kiwi_lidar
ls -l /dev/serial/by-id/
```

ESP32 DevKits and LD19 dongles both often use Silicon Labs CP2102 chips with
the same USB serial (`0001`). Identify the ESP32 by sending `PING` at 115200
and looking for `ACK,PONG`; do not assume `/dev/ttyUSB0` is the controller.
The installer then binds names to USB port paths, so keep each cable in the
same Pi socket.

Reinstall the udev rules with the ESP32 port first and LiDAR port second, then
reconnect devices and confirm group permissions:

```bash
bash scripts/install_udev_rules.sh /dev/ttyUSB0 /dev/ttyUSB1
```

Override launch/config ports only for diagnosis; stable names prevent swapped
controller and LiDAR ports.

## Bridge opens the port but does not move

- Send `PING` and look for `ACK,PONG`.
- Confirm `MODE,AUTO` was acknowledged and the gamepad software stop is clear.
- Confirm the ROS topic type is `geometry_msgs/msg/TwistStamped`.
- Check that commands arrive faster than the 250 ms ROS timeout and 300 ms
  ESP32 timeout.
- Test lifted wheels at low values. Never disable watchdogs to mask a rate
  problem.

## Wrong direction or odometry sign

Test one axis at a time. ROS expects forward +x, left +y, and
counter-clockwise +yaw. Compare command, wheel RPM, count direction, and
`/odom`. Correct wiring/direction configuration systematically; avoid stacking
multiple sign flips until motion appears right. Follow
[calibration.md](calibration.md).

## No scan or a rotating/offset map

- Confirm `/scan` rate and frame ID.
- Confirm one static `base_link -> base_laser` transform.
- Physically verify the provisional LiDAR translation and `-pi/2` yaw.
- Check timestamps and the full `map -> odom -> base_link -> base_laser` path.
- Stop mapping if scans do not overlay stationary walls.

## TF conflict

SLAM Toolbox and AMCL must not run together. The serial bridge owns
`odom -> base_link`; robot state publisher owns `base_link -> base_laser`.
Inspect `/tf` and `/tf_static` publisher details. Do not add another static
publisher to compensate for an incorrect URDF.

## Nav2 produces no command

Check lifecycle state, action status, costmaps, and transforms:

```bash
ros2 lifecycle nodes
ros2 action info /navigate_to_pose
ros2 topic echo /global_costmap/costmap --once
ros2 topic info /cmd_vel_nav --verbose
```

An invalid footprint, unavailable map, missing TF, or collision monitor can
correctly suppress motion. Resolve the cause instead of bypassing safety.

## Robot oscillates or tracks poorly

Return to lifted-wheel PID tests, then loaded wheel tests. Check battery
voltage, encoder counts/revolution, wheel radius, center distance, filter
alpha, breakaway PWM, loose hubs, and omni rollers. Record target/measured RPM
instead of tuning Nav2 around a motor-control fault.

## Build or dependency failure

Initialize submodules and apply repository patches:

```bash
git submodule update --init --recursive
bash scripts/apply_vendor_patches.sh
bash scripts/bootstrap_ros2.sh
```

Use Ubuntu 26.04/ROS 2 Lyrical. The GitHub workflow is the reference clean
container build. Do not remove third-party license headers while resolving
vendor issues.

## Useful evidence for a report

Include commit, OS/ROS version, launch command, firmware/toolchain versions,
hardware revision, full error text, and a minimal rosbag or serial excerpt.
Remove secrets and personal device identifiers first.
