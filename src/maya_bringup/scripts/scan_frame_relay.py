#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFrameRelay(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('frame_id', 'lidar_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(LaserScan, input_topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: LaserScan) -> None:
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
