# Mapping and navigation

The repository contains a plausible ROS 2 Lyrical SLAM/Nav2 stack, but it does
not contain evidence of a successful end-to-end physical navigation or
exploration run. Treat every autonomy launch as experimental.

## Prerequisites

- Complete electrical, firmware, command-timeout, encoder, geometry, and TF
  checks.
- Install a physical emergency power cutoff.
- Configure persistent `/dev/kiwi_esp32` and `/dev/kiwi_lidar` names.
- Measure the final footprint and `base_link -> base_laser` transform.
- Build with `./scripts/bootstrap_ros2.sh`.

## Modes

Manual mapping:

```bash
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true
```

Nav2 against the live SLAM map:

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_slam:=true enable_navigation:=true navigation_mode:=mapping
```

Saved-map localization (SLAM must remain off):

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_navigation:=true navigation_mode:=localization \
  map:=/absolute/path/kiwi_map.yaml
```

Experimental frontier exploration:

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_slam:=true enable_navigation:=true \
  navigation_mode:=mapping enable_exploration:=true
```

Do not run the last command at normal configured speed until reduced-speed
validation has passed.

## Staged hardware validation

The detailed source checklist is
[`src/kiwi_bringup/docs/pi_validation.md`](../src/kiwi_bringup/docs/pi_validation.md).
Execute it in order:

1. Build and run static checks.
2. Start hardware only; confirm scan, odometry, TF, and lifted-wheel command
   signs.
3. Build and save a map under manual supervision.
4. Start Nav2 in mapping mode and verify all velocity topics are
   `TwistStamped`.
5. Localize on a saved map and verify AMCL owns `map -> odom`.
6. Send sub-0.5 m forward and lateral goals in a clear bounded area.
7. Copy the Nav2 parameters, reduce translation to `0.10 m/s` and rotation to
   `0.35 rad/s`, then test exploration with a spotter.

At every stage:

```bash
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_laser
```

For navigation, `/cmd_vel` should have one publisher: collision monitor. Stop
if TF jumps, scans do not align, odometry signs are wrong, or another final
velocity publisher appears.

## Mapping quality

Drive slowly, maintain scan overlap, revisit known areas to close loops, and
avoid moving objects. A warped map usually indicates transform, timing,
odometry, or scan-origin problems—not a parameter that should be hidden by
larger tolerances.

Save a raster map for AMCL:

```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/KiwiDrive/maps/kiwi_map"
```

Set the initial pose in RViz before sending localization goals.

## Exploration safety gate

Use a closed room with no stairs or open exits, a 0.5 m obstacle buffer, a
second observer, and the physical cutoff in hand. Verify `explore_lite` sends
`NavigateToPose` goals and has no velocity publisher:

```bash
ros2 action info /navigate_to_pose
ros2 node info /explore_node
ros2 topic info /cmd_vel --verbose
```

Pause with:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
```

Wait for the robot to stop before shutting down. A test passes only when the
robot respects inflated obstacles, recovers or blacklists unreachable goals,
and preserves TF and velocity ownership. Record real evidence before updating
the project status.
