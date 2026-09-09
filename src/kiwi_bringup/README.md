# Kiwi Bringup

First-party ROS 2 Lyrical launch and configuration package for the physical
Kiwi-drive robot.

> This package is configured for staged integration. The repository does not
> yet contain evidence that Nav2 or frontier exploration has completed a
> physical-system validation run.

## Launch

```bash
ros2 launch kiwi_bringup hardware.launch.py
ros2 launch kiwi_bringup bringup.launch.py
ros2 launch kiwi_bringup slam.launch.py
ros2 launch kiwi_bringup exploration.launch.py
```

`hardware.launch.py` starts the robot description, ESP32 serial bridge, and
LD19 driver. It intentionally disables the description package's mock
`ros2_control` system and does not include the vendor LD19 launch file. The
`base_link` to `base_laser` transform is published only by
`robot_state_publisher` from the robot URDF.

`bringup.launch.py` includes the hardware stack exactly once. Autonomy is
opt-in:

```bash
# Mapping only
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true

# Navigate on the live SLAM map
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true enable_navigation:=true navigation_mode:=mapping

# Navigate on a saved raster map with AMCL
ros2 launch kiwi_bringup bringup.launch.py enable_navigation:=true navigation_mode:=localization map:=/absolute/path/kiwi_map.yaml

# Explore autonomously while building a map
ros2 launch kiwi_bringup bringup.launch.py enable_slam:=true enable_navigation:=true navigation_mode:=mapping enable_exploration:=true
```

Mapping mode uses SLAM Toolbox online async as the only `map -> odom`
publisher. Saved-map localization mode starts map_server and AMCL, configured
with `nav2_amcl::OmniMotionModel`; do not enable SLAM at the same time.

Nav2 is configured for DWB holonomic sampling in x, y, and yaw. Initial
controller limits are 0.22 m/s and 0.75 rad/s, below the bridge's configured
scaling maxima of 0.30 m/s and 1.0 rad/s. The stamped velocity path is:

`controller/behavior -> cmd_vel_nav -> velocity_smoother ->
cmd_vel_smoothed -> collision_monitor -> cmd_vel -> serial bridge`

All stages use `geometry_msgs/msg/TwistStamped`. This leaves one final
`cmd_vel` publisher and prevents the bridge from receiving unsmoothed commands.

## Autonomous exploration

Exploration uses `explore_lite` from the pinned
`robo-friends/m-explore-ros2` submodule. It reads
`/global_costmap/costmap` and `/global_costmap/costmap_updates`, whose static
layer follows the live `/map`, and sends goals to Nav2's
`/navigate_to_pose` action. It does not publish velocity commands; Nav2 remains
the only source upstream of the existing smoothed and collision-monitored
`cmd_vel` pipeline.

The bringup composition check only permits exploration when SLAM and Nav2 are
both enabled in `mapping` mode. An 8 second startup grace period lets their
lifecycle managers activate; after launch, `explore_lite` additionally waits
for the costmap, `map -> base_link` transform, and NavigateToPose action server.

In RViz, add a MarkerArray display on `/explore/frontiers`. Blue points are
candidate frontier cells, green spheres are candidate centroids, and red
frontiers have been blacklisted. A frontier is blacklisted when a Nav2 goal
aborts or makes no progress for 45 seconds; exploration then tries the next
reachable frontier and stops when none remain. The 0.75 m minimum frontier
size filters small gaps that are poor targets for the approximately 0.5 m
Kiwi chassis.

Pause and resume from another terminal:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

To stop safely, publish `false`, wait for Nav2 to report the goal canceled and
the robot to stop, then Ctrl-C the bringup. Keep the physical emergency stop
accessible; do not rely on Ctrl-C alone. Confirm `/cmd_vel` has only
`collision_monitor` as a publisher.

## Device names

The defaults expect persistent udev symlinks:

- `/dev/kiwi_esp32` for the ESP32
- `/dev/kiwi_lidar` for the LD19

Create host-specific udev rules before using these launch files. The LiDAR port
can be overridden with `laser_port:=...`; the controller port is configured in
`config/bridge_params.yaml`.

## LiDAR transform verification

The provisional `base_link` to `base_laser` scan origin is:

- translation: `(0.114, 0.0, 0.10977)` metres
- yaw: `-1.5707963267948966` radians (`-pi/2`)

These values preserve the existing physically intended patch transform, but
they require measurement and verification on the assembled robot before
mapping or navigation. They are exposed as `laser_x`, `laser_y`, `laser_z`,
and `laser_yaw` launch arguments.

## Footprint and Pi validation

The costmaps currently use a conservative provisional triangle for the
approximately 0.50 m chassis:
`[[0.30, 0.0], [-0.16, 0.28], [-0.16, -0.28]]`, plus 0.03 m padding.
Physically measure the complete assembled envelope and update both costmaps
before autonomous motion.

Follow the staged, command-by-command
[Raspberry Pi validation checklist](docs/pi_validation.md) before increasing
speed or sending normal navigation goals.
