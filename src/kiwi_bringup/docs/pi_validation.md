# Raspberry Pi 5 validation checklist

Run this checklist on Ubuntu 26.04 with ROS 2 Lyrical. Keep the robot on
blocks for the first command tests, then use an open floor with an accessible
physical emergency stop. In every terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/KiwiDrive/install/setup.bash
```

## 1. Build and static checks

```bash
cd ~/KiwiDrive
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to kiwi_bringup
source install/setup.bash
```

## 2. Hardware, TF, scan, and wheel odometry

```bash
ros2 launch kiwi_bringup hardware.launch.py
```

In separate terminals, verify the one-owner TF chain and sensor rates:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_laser
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Measure the assembled robot before floor operation. Confirm the provisional
`base_link -> base_laser` translation/yaw and the costmap triangle
`[[0.30, 0.0], [-0.16, 0.28], [-0.16, -0.28]]`, including wheels, wiring,
fasteners, and sensor overhang. Increase the footprint if any point lies
outside it.

With wheels clear, send each stamped command for only a few seconds, press
Ctrl-C, and immediately send the zero command:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.05}}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {y: 0.05}}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {angular: {z: 0.15}}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}}"
```

Confirm `/odom` signs match REP-103: forward is +x, left is +y, and
counter-clockwise is +yaw.

## 3. Build a map manually

Start hardware and online asynchronous SLAM:

```bash
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true
rviz2
```

Set RViz fixed frame to `map`; display Map (`/map`), LaserScan (`/scan`), and
TF. Drive slowly with the stamped commands above, keeping overlap and closing
loops. Verify the complete chain:

```bash
ros2 run tf2_ros tf2_echo map base_laser
ros2 topic hz /map
```

Save the raster map used by AMCL:

```bash
mkdir -p ~/KiwiDrive/maps
ros2 run nav2_map_server map_saver_cli -f ~/KiwiDrive/maps/kiwi_map
```

Optionally serialize the SLAM pose graph for later map continuation (this is
not the YAML/PGM map consumed by AMCL):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/KiwiDrive/maps/kiwi_map'}"
```

## 4. Navigate while mapping

```bash
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true enable_navigation:=true navigation_mode:=mapping
```

SLAM Toolbox owns `map -> odom`; AMCL and map_server must not be running.
Check the final velocity pipeline:

```bash
ros2 topic type /cmd_vel_nav
ros2 topic type /cmd_vel_smoothed
ros2 topic type /cmd_vel
ros2 topic info /cmd_vel --verbose
```

All three types must be `geometry_msgs/msg/TwistStamped`. `/cmd_vel` should
have one publisher (`collision_monitor`) and one subscriber
(`cmd_vel_serial_bridge`).

## 5. Localize on the saved map

Use an absolute map path. Do not enable SLAM in this mode:

```bash
ros2 launch kiwi_bringup bringup.launch.py enable_navigation:=true navigation_mode:=localization map:=$HOME/KiwiDrive/maps/kiwi_map.yaml
```

In RViz, set the initial pose with **2D Pose Estimate**, then verify:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /amcl_pose --once
```

## 6. Conservative goal tests

First send a 0.25 m forward goal in a known-clear area:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.25, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Then test a similarly short lateral goal to exercise Kiwi holonomy:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.25, z: 0.0}, orientation: {w: 1.0}}}}"
```

Use RViz goals after these pass. Keep initial tests below 0.5 m, watch the
local costmap and collision-monitor polygon, and tune only from recorded
`/scan`, `/odom`, `/cmd_vel`, and TF evidence.

## 7. Reduced-speed small-room exploration gate

Do not run this gate until sections 1-6 pass. Use a closed, uncluttered small
room with no stairs or open doors, establish a 0.5 m obstacle buffer, keep the
physical emergency stop in hand, and have a second person watch the robot.

Copy `nav2_params.yaml` to `nav2_params_small_room.yaml`. In the copy, reduce
DWB `FollowPath` values `min_vel_x/max_vel_x`, `min_vel_y/max_vel_y`, and
`max_speed_xy` to `-0.10/0.10`, and `max_vel_theta` to `0.35`. Also reduce
velocity_smoother `max_velocity` to `[0.10, 0.10, 0.35]` and `min_velocity` to
`[-0.10, -0.10, -0.35]`. Launch only with that reviewed file:

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_slam:=true \
  enable_navigation:=true \
  navigation_mode:=mapping \
  enable_exploration:=true \
  nav2_params_file:=$HOME/KiwiDrive/src/kiwi_bringup/config/nav2_params_small_room.yaml
```

The composition must be rejected if exploration is requested without either
SLAM or navigation, or with `navigation_mode:=localization`. During the
8-second startup delay, open RViz with fixed frame `map` and add Map (`/map`),
Map (`/global_costmap/costmap`), LaserScan (`/scan`), and MarkerArray
(`/explore/frontiers`).

Confirm that exploration sends Nav2 NavigateToPose goals and never publishes a
velocity topic:

```bash
ros2 action info /navigate_to_pose
ros2 topic info /cmd_vel --verbose
ros2 node info /explore_node
```

`/cmd_vel` must still have only `collision_monitor` as its publisher;
`/explore_node` must have no `cmd_vel` publisher. Verify the robot selects
frontiers, stays outside inflated obstacles, updates `/map`, and either reaches
or abandons each target. An aborted or no-progress target should be
blacklisted (red in RViz) after at most the configured 45 seconds, followed by
another reachable frontier; all unreachable frontiers should end exploration.

Test pause and resume while a goal is active:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

Pass only if pause cancels the active goal and the robot comes to a complete
stop, resume chooses a frontier without unsafe motion, and no collisions,
costmap violations, TF loss, or velocity-pipeline ownership changes occur.

To stop, publish `false`, wait until the robot is stationary, then Ctrl-C the
bringup. Use the physical emergency stop immediately for unexpected motion.
Save the map only after the robot is stopped. Do not raise the speed limits
until bagged `/scan`, `/odom`, `/map`, costmaps, TF, NavigateToPose status, and
the three velocity topics have been reviewed.
