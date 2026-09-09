# Architecture and ROS 2

This guide introduces the ROS concepts used by KiwiDrive and maps them to the
actual implementation. The target distribution is ROS 2 Lyrical.

> The checked-in `src/ROS2_Kiwi_Drive_Command_Guide.pdf` is legacy
> documentation. Its plain `geometry_msgs/msg/Twist` command examples are
> outdated. The current endpoint is `geometry_msgs/msg/TwistStamped`.

## ROS concepts in this project

A **node** is one running component. Nodes exchange typed **topics** (streams),
call **services** (request/response), and use **actions** for long-running
goals. **TF** is a time-indexed tree of coordinate frames. A valid navigation
system needs both sensor data and one unambiguous TF path from `map` to the
sensor.

Important first-party nodes:

- `cmd_vel_serial_bridge`: sends normalized commands to the ESP32, receives
  wheel telemetry, publishes `/odom`, and broadcasts `odom -> base_link`.
- `robot_state_publisher`: evaluates the Xacro model and publishes fixed/joint
  transforms, including `base_link -> base_laser`.
- Launch files in `kiwi_bringup` compose those nodes with LD19, SLAM Toolbox,
  Nav2, and optionally `explore_lite`.

Important external nodes:

- LD19 driver publishes `sensor_msgs/msg/LaserScan` on `/scan`.
- SLAM Toolbox consumes `/scan` plus TF, publishes `/map`, and owns
  `map -> odom` while mapping.
- AMCL owns `map -> odom` when localizing against a saved map.
- Nav2 plans and controls motion. Its collision monitor is intended to be the
  sole final publisher to `/cmd_vel`.
- `explore_lite` sends `NavigateToPose` actions to Nav2; it should never
  publish motor velocity directly.

## Command and feedback flow

```text
operator / Nav2 controller
  -> /cmd_vel_nav (TwistStamped)
  -> velocity_smoother
  -> /cmd_vel_smoothed (TwistStamped)
  -> collision_monitor
  -> /cmd_vel (TwistStamped)
  -> cmd_vel_serial_bridge
  -> "V,x,y,rotation\n" over USB serial
  -> ESP32 velocity PID
  -> three motor drivers

encoders
  -> ESP32 "ODOM,rpm1,rpm2,rpm3,count1,count2,count3\n"
  -> cmd_vel_serial_bridge
  -> /odom (nav_msgs/msg/Odometry)
  -> odom -> base_link TF
```

ROS uses REP-103 body axes: +x forward, +y left, +z up, and positive yaw
counter-clockwise. Firmware names normalized translation as `x` = robot right
and `y` = robot forward. The bridge performs that conversion; do not bypass it
with the legacy PDF convention.

## TF ownership

The intended tree is:

```text
map
  `-- odom                    SLAM Toolbox OR AMCL, never both
       `-- base_link          kiwi_serial_bridge
            |-- base_laser   robot_state_publisher
            `-- wheel_*      robot_state_publisher / joint states
```

There must be one publisher per dynamic edge. Running SLAM and AMCL together
creates two owners for `map -> odom` and is rejected by the bringup
composition. The LiDAR driver must not publish a duplicate static transform.

Inspect ownership:

```bash
ros2 topic info /tf --verbose
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_laser
ros2 run tf2_tools view_frames
```

## Launch layers

- `kiwi_drive_description display.launch.py`: visualization with mock
  `ros2_control`; never use it to operate the physical robot.
- `kiwi_bringup hardware.launch.py`: description, serial bridge, and LiDAR.
- `kiwi_bringup bringup.launch.py enable_slam:=true`: manual mapping.
- Add `enable_navigation:=true navigation_mode:=mapping` for Nav2 on a live
  map.
- Use `navigation_mode:=localization map:=/absolute/map.yaml` with SLAM off for
  saved-map AMCL.
- `enable_exploration:=true` is an experimental gate requiring live SLAM and
  Nav2; it is not yet validated on the robot.

See [Mapping and navigation](mapping-navigation.md) before enabling motion.
