# KiwiDrive Nav2 Quick Reference

> **Branch:** `feature/nav2-bringup`  
> **Safety:** Start with wheels raised. For floor tests, use a clear closed area, a spotter, and an accessible physical power cutoff. Press the physical cutoff for unexpected motion.

## 0. Open a terminal

```bash
cd ~/KiwiDrive
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

Run those four lines in **every new terminal**.

## 1. See the robot in RViz (no hardware motion)

```bash
ros2 launch kiwi_drive_description display.launch.py
```

This is static visualization only; it does not drive the robot.

## 2. Hardware + manual map in RViz

**Terminal 1 — start the robot and SLAM:**

```bash
cd ~/KiwiDrive
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true
```

**Terminal 2 — open RViz:**

```bash
source /opt/ros/lyrical/setup.bash
source ~/KiwiDrive/install/setup.bash
rviz2
```

In RViz set **Fixed Frame** to `map`, then add:

- **Map:** `/map`
- **LaserScan:** `/scan`
- **TF**
- **Odometry:** `/odom`

Drive slowly, overlap LiDAR scans, and revisit areas to close loops. Save the finished map:

```bash
mkdir -p ~/KiwiDrive/maps
ros2 run nav2_map_server map_saver_cli -f ~/KiwiDrive/maps/kiwi_map
```

## 3. Nav2 in a live SLAM map

Start this only after mapping and basic motion checks work.

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_slam:=true enable_navigation:=true navigation_mode:=mapping
```

Open RViz, keep Fixed Frame at `map`, and use **Nav2 Goal** / **2D Goal Pose** for short goals. First goals should be under 0.5 m.

## 4. Nav2 on a saved map

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_navigation:=true navigation_mode:=localization \
  map:=$HOME/KiwiDrive/maps/kiwi_map.yaml
```

In RViz: first click **2D Pose Estimate** to set the robot's starting position, then use **Nav2 Goal**.

## 5. Experimental frontier exploration

Only use after mapping, Nav2, and short-goal tests pass. Start at reduced speed in a small closed room.

```bash
ros2 launch kiwi_bringup bringup.launch.py \
  enable_slam:=true \
  enable_navigation:=true \
  navigation_mode:=mapping \
  enable_exploration:=true \
  nav2_params_file:=$HOME/KiwiDrive/src/kiwi_bringup/config/nav2_params_small_room.yaml
```

In RViz add:

- **Map:** `/map`
- **Map:** `/global_costmap/costmap`
- **LaserScan:** `/scan`
- **MarkerArray:** `/explore/frontiers`

Pause / resume exploration:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

## Manual motion tests (wheels raised first)

Run one motion command briefly, press `Ctrl-C`, then run the stop command.

**Forward:**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.05}}}"
```

**Left / sideways:**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {y: 0.05}}}"
```

**Rotate counter-clockwise:**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {angular: {z: 0.15}}}"
```

**Stop immediately after every test:**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}}"
```

## Quick health checks

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_laser
ros2 topic info /cmd_vel --verbose
```

For Nav2/exploration, `/cmd_vel` must have **one publisher only**: `collision_monitor`. Do not manually publish motion commands while Nav2 or exploration is running.
