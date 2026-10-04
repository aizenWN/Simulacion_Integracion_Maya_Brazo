#!/usr/bin/env python3

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def diagonal_covariance(values):
    cov = [0.0] * 36
    for index, value in values.items():
        cov[index] = float(value)
    return cov


def diagonal_covariance_3x3(values):
    cov = [0.0] * 9
    for index, value in values.items():
        cov[index] = float(value)
    return cov


class SimCovarianceRelay(Node):
    def __init__(self):
        super().__init__('sim_covariance_relay')

        self.declare_parameter('odom_input_topic', '/odom')
        self.declare_parameter('odom_output_topic', '/odom_with_covariance')
        self.declare_parameter('imu_input_topic', '/imu')
        self.declare_parameter('imu_output_topic', '/imu_with_covariance')

        odom_input = self.get_parameter('odom_input_topic').value
        odom_output = self.get_parameter('odom_output_topic').value
        imu_input = self.get_parameter('imu_input_topic').value
        imu_output = self.get_parameter('imu_output_topic').value

        # Indexes in 6x6 covariance matrices are:
        # [x, y, z, roll, pitch, yaw] and [vx, vy, vz, vroll, vpitch, vyaw].
        self._odom_pose_covariance = diagonal_covariance({
            0: 0.03,
            7: 0.03,
            14: 1e6,
            21: 1e6,
            28: 1e6,
            35: 0.20,
        })
        self._odom_twist_covariance = diagonal_covariance({
            0: 0.04,
            7: 0.10,
            14: 1e6,
            21: 1e6,
            28: 1e6,
            35: 0.08,
        })
        self._imu_orientation_covariance = diagonal_covariance_3x3({
            0: 1e6,
            4: 1e6,
            8: 0.02,
        })
        self._imu_angular_velocity_covariance = diagonal_covariance_3x3({
            0: 1e6,
            4: 1e6,
            8: 0.01,
        })
        self._imu_linear_acceleration_covariance = diagonal_covariance_3x3({
            0: 1e6,
            4: 1e6,
            8: 1e6,
        })

        # Publish rewritten topics with the default reliable QoS so EKF, diagnostics,
        # and generic ROS tools can all subscribe without QoS negotiation failures.
        self._odom_pub = self.create_publisher(Odometry, odom_output, 10)
        self._imu_pub = self.create_publisher(Imu, imu_output, 10)
        self.create_subscription(Odometry, odom_input, self._handle_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_input, self._handle_imu, qos_profile_sensor_data)

    def _handle_odom(self, msg: Odometry) -> None:
        out = copy.deepcopy(msg)
        out.pose.covariance = self._odom_pose_covariance
        out.twist.covariance = self._odom_twist_covariance
        self._odom_pub.publish(out)

    def _handle_imu(self, msg: Imu) -> None:
        out = copy.deepcopy(msg)
        out.orientation_covariance = self._imu_orientation_covariance
        out.angular_velocity_covariance = self._imu_angular_velocity_covariance
        out.linear_acceleration_covariance = self._imu_linear_acceleration_covariance
        self._imu_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SimCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
