#!/usr/bin/env python3
"""Publish map→odom correction TF from Gazebo ground truth.

Reads the robot's absolute pose from Gazebo via gz.transport (no ROS bridge),
computes the map→odom SE(2) correction, and publishes it as a TF transform.
Any controller that needs accurate RViz localization can rely on this node
without embedding TF logic in the controller itself.

  map ──(this node)──▶ odom ──(mecanum_drive_controller)──▶ base_footprint
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class GzPoseTfPublisher(Node):

    def __init__(self):
        super().__init__('gz_pose_tf_publisher')

        self.declare_parameter('robot_model_name', 'rosmaster_x3')
        self.declare_parameter('gz_world_name', 'factory_world')

        self._robot_model = self.get_parameter('robot_model_name').value
        gz_world = self.get_parameter('gz_world_name').value

        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._lock = threading.Lock()
        self._last_map_odom: TransformStamped | None = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(
            Odometry, '/mecanum_drive_controller/odom', self._odom_cb, 10)

        self._start_gz_subscriber(gz_world)

    def _start_gz_subscriber(self, gz_world: str):
        try:
            import gz.transport13 as gz_transport
            import gz.msgs10.pose_v_pb2 as pose_v_pb2

            self._gz_node = gz_transport.Node()
            topic = f'/world/{gz_world}/dynamic_pose/info'

            def _gz_pose_cb(msg):
                for pose in msg.pose:
                    if pose.name != self._robot_model:
                        continue
                    p = pose.position
                    o = pose.orientation
                    siny = 2.0 * (o.w * o.z + o.x * o.y)
                    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                    gz_yaw = math.atan2(siny, cosy)

                    with self._lock:
                        ox, oy, oyaw = self._odom_x, self._odom_y, self._odom_yaw

                    # T_map_odom = T_map_base * inv(T_odom_base)
                    cos_o = math.cos(oyaw); sin_o = math.sin(oyaw)
                    inv_x = -ox * cos_o - oy * sin_o
                    inv_y =  ox * sin_o - oy * cos_o
                    cos_g = math.cos(gz_yaw); sin_g = math.sin(gz_yaw)
                    mo_x   = p.x + inv_x * cos_g - inv_y * sin_g
                    mo_y   = p.y + inv_x * sin_g + inv_y * cos_g
                    mo_yaw = gz_yaw - oyaw

                    t = TransformStamped()
                    t.header.stamp.sec = msg.header.stamp.sec
                    t.header.stamp.nanosec = msg.header.stamp.nsec
                    t.header.frame_id = 'map'
                    t.child_frame_id = 'odom'
                    t.transform.translation.x = mo_x
                    t.transform.translation.y = mo_y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.z = math.sin(mo_yaw / 2.0)
                    t.transform.rotation.w = math.cos(mo_yaw / 2.0)
                    self._tf_broadcaster.sendTransform(t)
                    self._last_map_odom = t
                    break

            self._gz_node.subscribe(pose_v_pb2.Pose_V, topic, _gz_pose_cb)
            self.get_logger().info(f'Subscribed to Gazebo ground truth: {topic}')
        except Exception as e:
            self.get_logger().error(f'gz.transport failed: {e}')

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._odom_x   = msg.pose.pose.position.x
            self._odom_y   = msg.pose.pose.position.y
            self._odom_yaw = math.atan2(siny, cosy)

    def publish_static_last(self):
        if self._last_map_odom is not None:
            self._last_map_odom.header.stamp = self.get_clock().now().to_msg()
            self._static_tf_broadcaster.sendTransform(self._last_map_odom)


def main(args=None):
    rclpy.init(args=args)
    node = GzPoseTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_static_last()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
