#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def distance_2d(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


class Phase(Enum):
    WAITING_FOR_ODOM = "waiting_for_odom"
    PAUSE = "pause"
    DRIVE = "drive"
    TURN = "turn"
    COMPLETE = "complete"


class SquareTestNode(Node):
    def __init__(self) -> None:
        super().__init__("square_test")

        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("side_length", 1.0)
        self.declare_parameter("turn_angle_deg", 90.0)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("heading_gain", 1.0)
        self.declare_parameter("max_heading_correction", 0.2)
        self.declare_parameter("num_sides", 4)
        self.declare_parameter("pause_sec", 1.0)
        self.declare_parameter("control_rate_hz", 20.0)
        # Positive turn_angle_deg means a right turn, which normally uses negative
        # angular.z. Some SIM drive configurations invert the commanded yaw response;
        # set this to 1.0 for those tests without changing the desired yaw sign.
        self.declare_parameter("right_turn_angular_sign", -1.0)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.side_length = float(self.get_parameter("side_length").value)
        self.turn_angle = math.radians(float(self.get_parameter("turn_angle_deg").value))
        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.max_heading_correction = abs(float(self.get_parameter("max_heading_correction").value))
        self.num_sides = int(self.get_parameter("num_sides").value)
        self.pause_sec = float(self.get_parameter("pause_sec").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.right_turn_angular_sign = float(self.get_parameter("right_turn_angular_sign").value)
        if self.right_turn_angular_sign >= 0.0:
            self.right_turn_angular_sign = 1.0
        else:
            self.right_turn_angular_sign = -1.0

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_yaw: Optional[float] = None

        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None

        self.phase_start_x: Optional[float] = None
        self.phase_start_y: Optional[float] = None
        self.phase_start_yaw: Optional[float] = None
        self.target_drive_yaw: Optional[float] = None
        self.previous_turn_yaw: Optional[float] = None
        self.accumulated_turn_yaw = 0.0
        self.pause_deadline = None
        self.completed_sides = 0
        self.phase = Phase.WAITING_FOR_ODOM

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info(
            f"square_test using odom_topic={odom_topic}, cmd_vel_topic={cmd_vel_topic}, "
            f"side_length={self.side_length}, turn_angle_deg={math.degrees(self.turn_angle):.1f}, "
            f"linear_speed={self.linear_speed}, angular_speed={self.angular_speed}, "
            f"heading_gain={self.heading_gain}, max_heading_correction={self.max_heading_correction}, "
            f"num_sides={self.num_sides}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        self.current_yaw = quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def publish_stop(self) -> None:
        msg = Twist()
        self.cmd_pub.publish(msg)

    def publish_drive(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def begin_pause(self, next_phase: Phase) -> None:
        self.publish_stop()
        self.phase = Phase.PAUSE
        self.pause_deadline = self.get_clock().now().nanoseconds + int(self.pause_sec * 1e9)
        self.phase_after_pause = next_phase

    def begin_drive(self) -> None:
        self.phase = Phase.DRIVE
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y
        self.target_drive_yaw = self.current_yaw
        self.get_logger().info(
            f"Starting side {self.completed_sides + 1}/{self.num_sides} from "
            f"x={self.phase_start_x:.3f}, y={self.phase_start_y:.3f}, "
            f"target_yaw={math.degrees(self.target_drive_yaw):.1f} deg"
        )

    def begin_turn(self) -> None:
        self.phase = Phase.TURN
        self.phase_start_yaw = self.current_yaw
        self.previous_turn_yaw = self.current_yaw
        self.accumulated_turn_yaw = 0.0
        turn_direction = "right" if self.turn_angle >= 0.0 else "left"
        self.get_logger().info(
            f"Starting {turn_direction} turn {self.completed_sides + 1}/{self.num_sides} from "
            f"yaw={math.degrees(self.phase_start_yaw):.1f} deg"
        )

    def finish_test(self) -> None:
        self.publish_stop()
        self.phase = Phase.COMPLETE

        position_error = distance_2d(self.start_x, self.start_y, self.current_x, self.current_y)
        heading_error = math.degrees(wrap_angle(self.current_yaw - self.start_yaw))
        self.get_logger().info(
            f"Square test complete. Final pose x={self.current_x:.3f}, y={self.current_y:.3f}, "
            f"yaw={math.degrees(self.current_yaw):.1f} deg"
        )
        self.get_logger().info(
            f"Closure error: position={position_error:.3f} m, heading={heading_error:.1f} deg"
        )

    def control_loop(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.phase == Phase.WAITING_FOR_ODOM:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.get_logger().info(
                f"Initial pose x={self.start_x:.3f}, y={self.start_y:.3f}, "
                f"yaw={math.degrees(self.start_yaw):.1f} deg"
            )
            self.begin_pause(Phase.DRIVE)
            return

        if self.phase == Phase.PAUSE:
            if self.get_clock().now().nanoseconds >= self.pause_deadline:
                if self.phase_after_pause == Phase.DRIVE:
                    self.begin_drive()
                elif self.phase_after_pause == Phase.TURN:
                    self.begin_turn()
            return

        if self.phase == Phase.DRIVE:
            distance = distance_2d(self.phase_start_x, self.phase_start_y, self.current_x, self.current_y)
            if distance >= self.side_length:
                heading_error = wrap_angle(self.current_yaw - self.target_drive_yaw)
                self.get_logger().info(
                    f"Completed side {self.completed_sides + 1}/{self.num_sides}: "
                    f"distance={distance:.3f} m, heading_error={math.degrees(heading_error):.1f} deg"
                )
                self.begin_pause(Phase.TURN)
                return
            heading_error = wrap_angle(self.current_yaw - self.target_drive_yaw)
            heading_correction = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, -self.heading_gain * heading_error),
            )
            self.publish_drive(linear_x=self.linear_speed, angular_z=heading_correction)
            return

        if self.phase == Phase.TURN:
            yaw_step = wrap_angle(self.current_yaw - self.previous_turn_yaw)
            self.previous_turn_yaw = self.current_yaw
            self.accumulated_turn_yaw += yaw_step
            target_turn = abs(self.turn_angle)
            turn_complete = (
                self.accumulated_turn_yaw <= -target_turn
                if self.turn_angle >= 0.0
                else self.accumulated_turn_yaw >= target_turn
            )
            if turn_complete:
                self.completed_sides += 1
                self.get_logger().info(
                    f"Completed turn {self.completed_sides}/{self.num_sides}: "
                    f"yaw_delta={math.degrees(self.accumulated_turn_yaw):.1f} deg"
                )
                if self.completed_sides >= self.num_sides:
                    self.finish_test()
                else:
                    self.begin_pause(Phase.DRIVE)
                return
            angular_sign = self.right_turn_angular_sign if self.turn_angle >= 0.0 else -self.right_turn_angular_sign
            self.publish_drive(angular_z=angular_sign * self.angular_speed)
            return

        if self.phase == Phase.COMPLETE:
            self.publish_stop()


def main() -> None:
    rclpy.init()
    node = SquareTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
