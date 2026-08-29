#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial import SerialException
from tf2_ros import TransformBroadcaster


class CmdVelSerialBridge(Node):
    """Send velocity commands to the ESP32 and publish wheel odometry."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_serial_bridge")

        # ==================================================
        # ROS parameters
        # ==================================================

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.00)

        # Permanent robot geometry.
        self.declare_parameter("wheel_radius", 0.050)
        self.declare_parameter("wheel_distance", 0.2921)

        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("ros_command_timeout", 0.25)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # Permanent direction corrections.
        self.declare_parameter("odom_y_direction", -1.0)
        self.declare_parameter("odom_yaw_direction", 1.0)

        # ==================================================
        # Read parameters
        # ==================================================

        self.serial_port = (
            self.get_parameter("serial_port")
            .get_parameter_value()
            .string_value
        )

        self.baud_rate = (
            self.get_parameter("baud_rate")
            .get_parameter_value()
            .integer_value
        )

        self.max_linear_speed = (
            self.get_parameter("max_linear_speed")
            .get_parameter_value()
            .double_value
        )

        self.max_angular_speed = (
            self.get_parameter("max_angular_speed")
            .get_parameter_value()
            .double_value
        )

        self.wheel_radius = (
            self.get_parameter("wheel_radius")
            .get_parameter_value()
            .double_value
        )

        self.wheel_distance = (
            self.get_parameter("wheel_distance")
            .get_parameter_value()
            .double_value
        )

        self.command_rate_hz = (
            self.get_parameter("command_rate_hz")
            .get_parameter_value()
            .double_value
        )

        self.ros_command_timeout = (
            self.get_parameter("ros_command_timeout")
            .get_parameter_value()
            .double_value
        )

        self.odom_frame = (
            self.get_parameter("odom_frame")
            .get_parameter_value()
            .string_value
        )

        self.base_frame = (
            self.get_parameter("base_frame")
            .get_parameter_value()
            .string_value
        )

        self.odom_y_direction = (
            self.get_parameter("odom_y_direction")
            .get_parameter_value()
            .double_value
        )

        self.odom_yaw_direction = (
            self.get_parameter("odom_yaw_direction")
            .get_parameter_value()
            .double_value
        )

        # ==================================================
        # Validate parameters
        # ==================================================

        if self.max_linear_speed <= 0.0:
            raise ValueError("max_linear_speed must be greater than zero")

        if self.max_angular_speed <= 0.0:
            raise ValueError("max_angular_speed must be greater than zero")

        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be greater than zero")

        if self.wheel_distance <= 0.0:
            raise ValueError("wheel_distance must be greater than zero")

        if self.command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be greater than zero")

        # ==================================================
        # Serial connection
        # ==================================================

        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.02,
                write_timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

        except SerialException as error:
            self.get_logger().fatal(
                f"Could not open {self.serial_port}: {error}"
            )
            raise

        time.sleep(2.0)

        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

        self.serial_lock = threading.Lock()

        # ==================================================
        # Velocity-command state
        # ==================================================

        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_rotation = 0.0

        self.last_cmd_vel_time = self.get_clock().now()

        # ==================================================
        # Odometry state
        # ==================================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.last_odom_time = None
        self.odom_packet_count = 0

        # ==================================================
        # ROS interfaces
        # ==================================================

        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            20,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self.send_latest_command,
        )

        self.serial_read_timer = self.create_timer(
            0.01,
            self.read_serial,
        )

        self.send_line("MODE,AUTO")

        self.get_logger().info(
            f"Connected to ESP32 on {self.serial_port} "
            f"at {self.baud_rate} baud"
        )

        self.get_logger().info(
            "Kiwi geometry: "
            f"wheel radius={self.wheel_radius:.4f} m, "
            f"center-to-wheel={self.wheel_distance:.4f} m"
        )

    # ======================================================
    # Utility functions
    # ======================================================

    @staticmethod
    def clamp(value: float) -> float:
        return max(-1.0, min(value, 1.0))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def yaw_to_quaternion(
        yaw: float,
    ) -> tuple[float, float, float, float]:
        half_yaw = yaw * 0.5

        return (
            0.0,
            0.0,
            math.sin(half_yaw),
            math.cos(half_yaw),
        )

    # ======================================================
    # ROS cmd_vel handling
    # ======================================================

    def cmd_vel_callback(
        self,
        message: TwistStamped,
    ) -> None:
        """
        ROS:
          linear.x  = forward
          linear.y  = left
          angular.z = counterclockwise

        ESP32:
          x        = robot sideways axis
          y        = robot forward axis
          rotation = robot rotation
        """

        # Positive ROS left now maps to the correct ESP32
        # sideways direction.
        self.latest_x = self.clamp(
            message.twist.linear.y
            / self.max_linear_speed
        )

        self.latest_y = self.clamp(
            message.twist.linear.x
            / self.max_linear_speed
        )

        self.latest_rotation = self.clamp(
            message.twist.angular.z
            / self.max_angular_speed
        )

        self.last_cmd_vel_time = self.get_clock().now()

    def send_latest_command(self) -> None:
        command_age = (
            self.get_clock().now()
            - self.last_cmd_vel_time
        ).nanoseconds / 1e9

        if command_age > self.ros_command_timeout:
            x = 0.0
            y = 0.0
            rotation = 0.0
        else:
            x = self.latest_x
            y = self.latest_y
            rotation = self.latest_rotation

        self.send_line(
            f"V,{x:.4f},{y:.4f},{rotation:.4f}"
        )

    # ======================================================
    # Serial communication
    # ======================================================

    def send_line(self, command: str) -> None:
        data = f"{command}\n".encode("ascii")

        try:
            with self.serial_lock:
                self.serial_connection.write(data)
                self.serial_connection.flush()

        except SerialException as error:
            self.get_logger().error(
                f"Serial write failed: {error}"
            )

    def read_serial(self) -> None:
        try:
            while self.serial_connection.in_waiting > 0:
                with self.serial_lock:
                    raw_line = self.serial_connection.readline()

                line = raw_line.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                if not line:
                    continue

                if line.startswith("ODOM,"):
                    self.process_odom_line(line)

                elif (
                    line.startswith("WARN")
                    or line.startswith("ERR")
                    or line.startswith("MODE")
                ):
                    self.get_logger().info(line)

        except SerialException as error:
            self.get_logger().error(
                f"Serial read failed: {error}"
            )

    # ======================================================
    # Odometry processing
    # ======================================================

    def process_odom_line(self, line: str) -> None:
        """
        Expected ESP32 format:

        ODOM,rpm1,rpm2,rpm3,count1,count2,count3
        """

        fields = line.split(",")

        if len(fields) != 7:
            self.get_logger().warning(
                f"Invalid ODOM field count: {line}"
            )
            return

        try:
            rpm1 = float(fields[1])
            rpm2 = float(fields[2])
            rpm3 = float(fields[3])

            int(fields[4])
            int(fields[5])
            int(fields[6])

        except ValueError:
            self.get_logger().warning(
                f"Invalid ODOM values: {line}"
            )
            return

        self.odom_packet_count += 1

        if self.odom_packet_count % 20 == 0:
            self.get_logger().info(
                f"Receiving odometry: {line}"
            )

        current_time = self.get_clock().now()

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = (
            current_time - self.last_odom_time
        ).nanoseconds / 1e9

        self.last_odom_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        rpm_to_rad_per_second = (
            2.0 * math.pi / 60.0
        )

        wheel1_velocity = (
            rpm1
            * rpm_to_rad_per_second
            * self.wheel_radius
        )

        wheel2_velocity = (
            rpm2
            * rpm_to_rad_per_second
            * self.wheel_radius
        )

        wheel3_velocity = (
            rpm3
            * rpm_to_rad_per_second
            * self.wheel_radius
        )

        # Inverse of the corrected ESP32 Kiwi equations.

        robot_right_velocity = (
            2.0 * wheel2_velocity
            - wheel1_velocity
            - wheel3_velocity
        ) / 3.0

        robot_forward_velocity = (
            wheel1_velocity
            - wheel3_velocity
        ) / math.sqrt(3.0)

        robot_angular_velocity = (
            wheel1_velocity
            + wheel2_velocity
            + wheel3_velocity
        ) / (
            3.0 * self.wheel_distance
        )

        # ROS base_link convention:
        # +x = forward
        # +y = left
        # +yaw = counterclockwise

        vx = robot_forward_velocity

        vy = (
            -robot_right_velocity
            * self.odom_y_direction
        )

        wz = (
            robot_angular_velocity
            * self.odom_yaw_direction
        )

        cos_yaw = math.cos(self.odom_yaw)
        sin_yaw = math.sin(self.odom_yaw)

        world_vx = (
            vx * cos_yaw
            - vy * sin_yaw
        )

        world_vy = (
            vx * sin_yaw
            + vy * cos_yaw
        )

        self.odom_x += world_vx * dt
        self.odom_y += world_vy * dt
        self.odom_yaw += wz * dt

        self.odom_yaw = self.normalize_angle(
            self.odom_yaw
        )

        self.publish_odometry(
            current_time,
            vx,
            vy,
            wz,
        )

    # ======================================================
    # Publish odometry + TF
    # ======================================================

    def publish_odometry(
        self,
        timestamp,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        qx, qy, qz, qw = self.yaw_to_quaternion(
            self.odom_yaw
        )

        odom_message = Odometry()

        odom_message.header.stamp = timestamp.to_msg()
        odom_message.header.frame_id = self.odom_frame
        odom_message.child_frame_id = self.base_frame

        odom_message.pose.pose.position.x = self.odom_x
        odom_message.pose.pose.position.y = self.odom_y
        odom_message.pose.pose.position.z = 0.0

        odom_message.pose.pose.orientation.x = qx
        odom_message.pose.pose.orientation.y = qy
        odom_message.pose.pose.orientation.z = qz
        odom_message.pose.pose.orientation.w = qw

        odom_message.twist.twist.linear.x = vx
        odom_message.twist.twist.linear.y = vy
        odom_message.twist.twist.linear.z = 0.0

        odom_message.twist.twist.angular.x = 0.0
        odom_message.twist.twist.angular.y = 0.0
        odom_message.twist.twist.angular.z = wz

        odom_message.pose.covariance[0] = 0.05
        odom_message.pose.covariance[7] = 0.05
        odom_message.pose.covariance[14] = 99999.0
        odom_message.pose.covariance[21] = 99999.0
        odom_message.pose.covariance[28] = 99999.0
        odom_message.pose.covariance[35] = 0.10

        odom_message.twist.covariance[0] = 0.05
        odom_message.twist.covariance[7] = 0.05
        odom_message.twist.covariance[14] = 99999.0
        odom_message.twist.covariance[21] = 99999.0
        odom_message.twist.covariance[28] = 99999.0
        odom_message.twist.covariance[35] = 0.10

        self.odom_publisher.publish(odom_message)

        transform = TransformStamped()

        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    # ======================================================
    # Shutdown
    # ======================================================

    def stop_robot(self) -> None:
        try:
            self.send_line("STOP")
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self.stop_robot()

        try:
            self.send_line("MODE,MANUAL")
            self.serial_connection.close()

        except SerialException:
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CmdVelSerialBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()