#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import LaserScan


class ScanMerge(Node):
    def __init__(self) -> None:
        super().__init__('scan_merger')
        self.declare_parameter('scan_topic_1', '/scan_fixed')
        self.declare_parameter('scan_topic_2', '/scan_depth')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('frame_id', 'base_footprint')

        self._scan_topic_1 = self.get_parameter('scan_topic_1').value
        self._scan_topic_2 = self.get_parameter('scan_topic_2').value
        self._output_topic = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(LaserScan, self._output_topic, qos_profile_sensor_data)
        self._sub_1 = Subscriber(self, LaserScan, self._scan_topic_1, qos_profile=qos_profile_sensor_data)
        self._sub_2 = Subscriber(self, LaserScan, self._scan_topic_2, qos_profile=qos_profile_sensor_data)
        self._sync = TimeSynchronizer([self._sub_1, self._sub_2], queue_size=20)
        self._sync.registerCallback(self._sync_cb)

    def _sync_cb(self, scan_1: LaserScan, scan_2: LaserScan) -> None:
        primary, secondary = scan_1, scan_2

        merged = LaserScan()
        merged.header = primary.header
        merged.header.frame_id = self._frame_id
        merged.angle_min = primary.angle_min
        merged.angle_max = primary.angle_max
        merged.angle_increment = primary.angle_increment
        merged.time_increment = primary.time_increment
        merged.scan_time = primary.scan_time
        merged.range_min = min(primary.range_min, secondary.range_min)
        merged.range_max = max(primary.range_max, secondary.range_max)

        merged.ranges = self._merge_ranges(primary, secondary)
        merged.intensities = list(primary.intensities)

        self._pub.publish(merged)

    @staticmethod
    def _merge_ranges(primary: LaserScan, secondary: LaserScan) -> list[float]:
        merged: list[float] = []
        sec_min = secondary.angle_min
        sec_inc = secondary.angle_increment
        sec_count = len(secondary.ranges)
        tol = 1.0e-4

        for idx, r1 in enumerate(primary.ranges):
            angle = primary.angle_min + idx * primary.angle_increment
            sec_idx_f = (angle - sec_min) / sec_inc
            sec_idx = int(round(sec_idx_f))

            r2 = math.inf
            if 0 <= sec_idx < sec_count:
                angle_err = abs(sec_idx_f - sec_idx)
                if angle_err <= tol:
                    r2 = secondary.ranges[sec_idx]

            merged.append(ScanMerge._merge_range(r1, r2))

        return merged

    @staticmethod
    def _merge_range(r1: float, r2: float) -> float:
        v1 = r1 if math.isfinite(r1) and r1 > 0.0 else math.inf
        v2 = r2 if math.isfinite(r2) and r2 > 0.0 else math.inf
        return min(v1, v2)


def main() -> None:
    rclpy.init()
    node = ScanMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
