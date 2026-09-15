"""ROS 2 node connecting TwistStamped commands to the Kiwi ESP32."""

from typing import List, Optional

from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from serial import SerialException
from tf2_ros import TransformBroadcaster

from .kiwi_kinematics import (
    BodyVelocity,
    command_for_age,
    twist_to_command,
    wheel_rpm_to_body_velocity,
)
from .odometry import Pose2D, integrate_pose, yaw_to_quaternion
from .serial_protocol import (
    AckMessage,
    ErrMessage,
    ModeCommand,
    OdomMessage,
    StopCommand,
    VelocityCommand,
    WarnMessage,
    format_message,
    parse_message,
)
from .serial_transport import SerialTransport


class CmdVelSerialBridge(Node):
    """Send velocity commands to the ESP32 and publish wheel odometry."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_serial_bridge")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        try:
            self.transport = SerialTransport(
                port=self.serial_port,
                baud_rate=self.baud_rate,
                read_timeout=self.serial_read_timeout,
                write_timeout=self.serial_write_timeout,
                startup_delay=self.serial_startup_delay,
            )
        except SerialException as error:
            self.get_logger().fatal(
                f"Could not open {self.serial_port}: {error}"
            )
            raise

        self.latest_command = VelocityCommand(0.0, 0.0, 0.0)
        self.last_cmd_vel_time = self.get_clock().now()
        self.pose = Pose2D()
        self.last_odom_time: Optional[Time] = None
        self.odom_packet_count = 0
        self._transport_closed = False

        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self.send_latest_command,
        )
        self.serial_read_timer = self.create_timer(
            1.0 / self.serial_read_rate_hz,
            self.read_serial,
        )
        self._odom_watchdog_timer = self.create_timer(
            2.0,
            self._warn_if_no_odometry,
        )

        self._send_message(ModeCommand("AUTO"))
        self.get_logger().info(
            f"Connected to ESP32 on {self.serial_port} "
            f"at {self.baud_rate} baud"
        )
        self.get_logger().info(
            "Kiwi geometry: "
            f"wheel radius={self.wheel_radius:.4f} m, "
            f"center-to-wheel={self.wheel_distance:.4f} m"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("serial_port", "/dev/kiwi_esp32")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_read_timeout", 0.02)
        self.declare_parameter("serial_write_timeout", 0.1)
        self.declare_parameter("serial_startup_delay", 2.0)
        self.declare_parameter("serial_read_rate_hz", 100.0)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.00)
        self.declare_parameter("wheel_radius", 0.050)
        self.declare_parameter("wheel_distance", 0.2921)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("ros_command_timeout", 0.25)
        self.declare_parameter("max_odom_dt", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("command_sideways_direction", 1.0)
        self.declare_parameter("command_forward_direction", 1.0)
        self.declare_parameter("command_rotation_direction", 1.0)
        self.declare_parameter("odom_y_direction", -1.0)
        self.declare_parameter("odom_yaw_direction", 1.0)
        self.declare_parameter(
            "pose_covariance_diagonal",
            [0.05, 0.05, 99999.0, 99999.0, 99999.0, 0.10],
        )
        self.declare_parameter(
            "twist_covariance_diagonal",
            [0.05, 0.05, 99999.0, 99999.0, 99999.0, 0.10],
        )

    def _read_parameters(self) -> None:
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.serial_read_timeout = float(
            self.get_parameter("serial_read_timeout").value
        )
        self.serial_write_timeout = float(
            self.get_parameter("serial_write_timeout").value
        )
        self.serial_startup_delay = float(
            self.get_parameter("serial_startup_delay").value
        )
        self.serial_read_rate_hz = float(
            self.get_parameter("serial_read_rate_hz").value
        )
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_distance = float(
            self.get_parameter("wheel_distance").value
        )
        self.command_rate_hz = float(
            self.get_parameter("command_rate_hz").value
        )
        self.ros_command_timeout = float(
            self.get_parameter("ros_command_timeout").value
        )
        self.max_odom_dt = float(self.get_parameter("max_odom_dt").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.command_sideways_direction = float(
            self.get_parameter("command_sideways_direction").value
        )
        self.command_forward_direction = float(
            self.get_parameter("command_forward_direction").value
        )
        self.command_rotation_direction = float(
            self.get_parameter("command_rotation_direction").value
        )
        self.odom_y_direction = float(
            self.get_parameter("odom_y_direction").value
        )
        self.odom_yaw_direction = float(
            self.get_parameter("odom_yaw_direction").value
        )
        self.pose_covariance_diagonal = [
            float(value)
            for value in self.get_parameter(
                "pose_covariance_diagonal"
            ).value
        ]
        self.twist_covariance_diagonal = [
            float(value)
            for value in self.get_parameter(
                "twist_covariance_diagonal"
            ).value
        ]

    def _validate_parameters(self) -> None:
        positive = {
            "baud_rate": self.baud_rate,
            "serial_read_rate_hz": self.serial_read_rate_hz,
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "wheel_radius": self.wheel_radius,
            "wheel_distance": self.wheel_distance,
            "command_rate_hz": self.command_rate_hz,
            "ros_command_timeout": self.ros_command_timeout,
            "max_odom_dt": self.max_odom_dt,
        }
        for name, value in positive.items():
            if value <= 0:
                raise ValueError(f"{name} must be greater than zero")
        for name, value in {
            "serial_read_timeout": self.serial_read_timeout,
            "serial_write_timeout": self.serial_write_timeout,
            "serial_startup_delay": self.serial_startup_delay,
        }.items():
            if value < 0:
                raise ValueError(f"{name} must not be negative")
        for name, values in {
            "pose_covariance_diagonal": self.pose_covariance_diagonal,
            "twist_covariance_diagonal": self.twist_covariance_diagonal,
        }.items():
            if len(values) != 6:
                raise ValueError(f"{name} must contain six values")

    def cmd_vel_callback(self, message: TwistStamped) -> None:
        """Store the latest normalized command."""
        self.latest_command = twist_to_command(
            linear_x=message.twist.linear.x,
            linear_y=message.twist.linear.y,
            angular_z=message.twist.angular.z,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            sideways_direction=self.command_sideways_direction,
            forward_direction=self.command_forward_direction,
            rotation_direction=self.command_rotation_direction,
        )
        self.last_cmd_vel_time = self.get_clock().now()

    def send_latest_command(self) -> None:
        """Send the current command, applying the ROS command timeout."""
        age = (
            self.get_clock().now() - self.last_cmd_vel_time
        ).nanoseconds / 1e9
        self._send_message(
            command_for_age(
                self.latest_command,
                age,
                self.ros_command_timeout,
            )
        )

    def _send_message(
        self,
        message: VelocityCommand | ModeCommand | StopCommand,
    ) -> None:
        try:
            self.transport.write_line(format_message(message))
        except SerialException as error:
            self.get_logger().error(f"Serial write failed: {error}")

    def read_serial(self) -> None:
        """Read and dispatch all available controller messages."""
        try:
            lines = self.transport.read_available()
        except SerialException as error:
            self.get_logger().error(f"Serial read failed: {error}")
            return

        for line in lines:
            try:
                message = parse_message(line)
            except ValueError:
                if line.startswith("ODOM,"):
                    self.get_logger().warning(f"Invalid ODOM values: {line}")
                continue

            if isinstance(message, OdomMessage):
                self._process_odometry(message, line)
            elif isinstance(message, AckMessage):
                if not message.detail.startswith("V,"):
                    self.get_logger().info(line)
            elif isinstance(
                message,
                (WarnMessage, ErrMessage, ModeCommand),
            ):
                self.get_logger().info(line)

    def _warn_if_no_odometry(self) -> None:
        """Surface a silent serial RX failure instead of a missing TF."""
        if self.odom_packet_count == 0:
            self.get_logger().warning(
                "No ODOM lines received from the ESP32 yet"
            )

    def _process_odometry(
        self,
        message: OdomMessage,
        original_line: str,
    ) -> None:
        self.odom_packet_count += 1
        if self.odom_packet_count % 20 == 0:
            self.get_logger().info(
                f"Receiving odometry: {original_line}"
            )

        current_time = self.get_clock().now()
        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        if dt <= 0.0 or dt > self.max_odom_dt:
            return

        velocity = wheel_rpm_to_body_velocity(
            message.rpm1,
            message.rpm2,
            message.rpm3,
            self.wheel_radius,
            self.wheel_distance,
            self.odom_y_direction,
            self.odom_yaw_direction,
        )
        self.pose = integrate_pose(self.pose, velocity, dt)
        self._publish_odometry(current_time, velocity)

    def _publish_odometry(
        self,
        timestamp: Time,
        velocity: BodyVelocity,
    ) -> None:
        qx, qy, qz, qw = yaw_to_quaternion(self.pose.yaw)
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity.vx
        odom.twist.twist.linear.y = velocity.vy
        odom.twist.twist.angular.z = velocity.wz
        self._set_covariance(
            odom.pose.covariance,
            self.pose_covariance_diagonal,
        )
        self._set_covariance(
            odom.twist.covariance,
            self.twist_covariance_diagonal,
        )
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _set_covariance(
        covariance: List[float],
        diagonal: List[float],
    ) -> None:
        for index, value in zip((0, 7, 14, 21, 28, 35), diagonal):
            covariance[index] = value

    def destroy_node(self) -> None:
        """Stop the controller and close serial before ROS teardown."""
        if not self._transport_closed:
            self._send_message(StopCommand())
            self._send_message(ModeCommand("MANUAL"))
            try:
                self.transport.close()
            except SerialException:
                pass
            self._transport_closed = True
        super().destroy_node()
