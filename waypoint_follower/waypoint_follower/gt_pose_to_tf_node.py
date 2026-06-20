#!/usr/bin/env python3
"""Convert Gazebo world poses into a normalized ground-truth odometry TF."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthPoseToTf(Node):

    def __init__(self):
        super().__init__('gt_pose_to_tf')

        self.declare_parameter('input_topic', '/gz_world_poses')
        self.declare_parameter('transform_index', 0)
        self.declare_parameter('parent_frame', 'gt_odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/gt/odom')
        self.declare_parameter('path_topic', '/gt/path')
        self.declare_parameter('path_min_distance', 0.02)

        self.input_topic = self.get_parameter('input_topic').value
        self.transform_index = int(
            self.get_parameter('transform_index').value)
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.path_min_distance = float(
            self.get_parameter('path_min_distance').value)

        self.initial_pose = None
        self.last_path_xy = None
        self.path = Path()
        self.path.header.frame_id = self.parent_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            TFMessage, self.input_topic, self.pose_callback, qos)

        self.get_logger().info(
            f'Converting {self.input_topic}[{self.transform_index}] to '
            f'TF {self.parent_frame}->{self.child_frame}')

    def pose_callback(self, msg: TFMessage) -> None:
        if self.transform_index >= len(msg.transforms):
            self.get_logger().warning(
                f'GT transform index {self.transform_index} is unavailable; '
                f'message has {len(msg.transforms)} transforms',
                throttle_duration_sec=5.0)
            return

        source = msg.transforms[self.transform_index].transform
        world_x = source.translation.x
        world_y = source.translation.y
        world_yaw = yaw_from_quaternion(source.rotation)

        if self.initial_pose is None:
            self.initial_pose = (world_x, world_y, world_yaw)
            self.get_logger().info(
                'GT origin initialized at '
                f'x={world_x:.3f}, y={world_y:.3f}, '
                f'yaw={math.degrees(world_yaw):.3f} deg')

        x0, y0, yaw0 = self.initial_pose
        dx = world_x - x0
        dy = world_y - y0
        cos0 = math.cos(yaw0)
        sin0 = math.sin(yaw0)
        x = cos0 * dx + sin0 * dy
        y = -sin0 * dx + cos0 * dy
        yaw = normalize_angle(world_yaw - yaw0)

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 1.0e-8
        odom.pose.covariance[7] = 1.0e-8
        odom.pose.covariance[35] = 1.0e-8
        self.odom_pub.publish(odom)

        if self.should_append_path(x, y):
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose = odom.pose.pose
            self.path.poses.append(pose)
            self.last_path_xy = (x, y)

        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)

    def should_append_path(self, x: float, y: float) -> bool:
        if self.last_path_xy is None:
            return True
        return math.hypot(
            x - self.last_path_xy[0],
            y - self.last_path_xy[1]) >= self.path_min_distance


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPoseToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
