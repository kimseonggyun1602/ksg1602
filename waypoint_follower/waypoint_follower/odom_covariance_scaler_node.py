#!/usr/bin/env python3
"""Apply conservative covariance floors to an odometry stream."""

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomCovarianceScaler(Node):
    """Keep externally estimated odometry from dominating an EKF update."""

    def __init__(self):
        super().__init__('odom_covariance_scaler')
        self.declare_parameter('input_topic', '/icp/odom_raw')
        self.declare_parameter('output_topic', '/icp/odom')
        self.declare_parameter('pose_xy_variance_floor', 0.020)
        self.declare_parameter('pose_yaw_variance_floor', 0.015)
        self.declare_parameter('twist_xy_variance_floor', 0.040)
        self.declare_parameter('twist_yaw_variance_floor', 0.020)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self.callback, 10)
        self.get_logger().info(
            f'Odometry covariance floors: {input_topic} -> {output_topic}')

    def callback(self, raw_msg: Odometry) -> None:
        msg = copy.deepcopy(raw_msg)
        pose_xy = self.get_parameter('pose_xy_variance_floor').value
        pose_yaw = self.get_parameter('pose_yaw_variance_floor').value
        twist_xy = self.get_parameter('twist_xy_variance_floor').value
        twist_yaw = self.get_parameter('twist_yaw_variance_floor').value

        msg.pose.covariance[0] = max(msg.pose.covariance[0], pose_xy)
        msg.pose.covariance[7] = max(msg.pose.covariance[7], pose_xy)
        msg.pose.covariance[35] = max(msg.pose.covariance[35], pose_yaw)
        msg.twist.covariance[0] = max(msg.twist.covariance[0], twist_xy)
        msg.twist.covariance[7] = max(msg.twist.covariance[7], twist_xy)
        msg.twist.covariance[35] = max(
            msg.twist.covariance[35], twist_yaw)
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceScaler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
