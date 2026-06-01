#!/usr/bin/env python3
"""Record raw and degraded wheel odometry paths for saved-result comparisons."""

import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WheelPathRecorder(Node):
    def __init__(self):
        super().__init__('wheel_path_recorder')
        self.declare_parameter('results_dir', '')
        self.declare_parameter(
            'raw_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('degraded_topic', '/wheel_odom/degraded')

        results_dir = os.path.expanduser(
            self.get_parameter('results_dir').value)
        os.makedirs(results_dir, exist_ok=True)
        self.files = []
        self.writers = {}
        for key, filename in (
            ('raw', 'raw_wheel_odom.csv'),
            ('degraded', 'degraded_wheel_odom.csv'),
        ):
            output = open(
                os.path.join(results_dir, filename), 'w',
                newline='', encoding='utf-8')
            writer = csv.writer(output)
            writer.writerow(['stamp_sec', 'x_m', 'y_m', 'yaw_rad'])
            self.files.append(output)
            self.writers[key] = writer

        self.create_subscription(
            Odometry, self.get_parameter('raw_topic').value,
            lambda msg: self.record('raw', msg), 50)
        self.create_subscription(
            Odometry, self.get_parameter('degraded_topic').value,
            lambda msg: self.record('degraded', msg), 50)
        self.get_logger().info(
            f'Recording raw and degraded wheel paths to {results_dir}')

    def record(self, key: str, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.writers[key].writerow([stamp, position.x, position.y, yaw])

    def close(self) -> None:
        for output in self.files:
            output.close()


def main(args=None):
    rclpy.init(args=args)
    node = WheelPathRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
