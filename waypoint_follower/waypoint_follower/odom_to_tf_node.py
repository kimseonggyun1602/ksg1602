#!/usr/bin/env python3
"""Republish nav_msgs/Odometry poses as a TF transform."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    """Provide the TF interface expected by SLAM Toolbox during bag replay."""

    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter(
            'input_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')

        input_topic = self.get_parameter('input_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, input_topic, self.odom_callback, 50)
        self.get_logger().info(
            f'Publishing TF {self.parent_frame} -> {self.child_frame} '
            f'from {input_topic}')

    def odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
