#!/usr/bin/env python3
"""Waypoint follower for Yahboom ROSMASTER X3 mecanum-wheel robot.

Coordinate frames:
  - Waypoints:  Gazebo world frame (absolute x, y)
  - Robot pose: from gz.transport directly (/world/.../dynamic_pose/info)
                → absolute world-frame position, bypasses odom entirely
  - cmd_vel:    robot body frame (x forward, y left)
  - Conversion: rotate world-frame error by -yaw
      vx =  dx·cos θ + dy·sin θ
      vy = -dx·sin θ + dy·cos θ
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('use_ground_truth', True)
        self.declare_parameter('robot_model_name', 'rosmaster_x3')
        self.declare_parameter('gz_world_name', 'factory_world')
        self.declare_parameter('exit_after_completion', False)
        self.declare_parameter('fixed_heading_enabled', False)
        self.declare_parameter('fixed_heading_yaw', 0.0)

        wp_file          = self.get_parameter('waypoints_file').value
        self.tol         = self.get_parameter('goal_tolerance').value
        self.max_v       = self.get_parameter('max_linear_vel').value
        self.max_w       = self.get_parameter('max_angular_vel').value
        self.kp_l        = self.get_parameter('kp_linear').value
        self.kp_a        = self.get_parameter('kp_angular').value
        use_gt           = self.get_parameter('use_ground_truth').value
        self.robot_model = self.get_parameter('robot_model_name').value
        gz_world         = self.get_parameter('gz_world_name').value
        self.exit_after_completion = self.get_parameter(
            'exit_after_completion').value
        self.fixed_heading_enabled = self.get_parameter(
            'fixed_heading_enabled').value
        self.fixed_heading_yaw = self.get_parameter(
            'fixed_heading_yaw').value

        self.waypoints  = self._load_waypoints(wp_file)
        self.wp_idx     = 0
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.pose_ready = False
        self._lock      = threading.Lock()

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/cmd_vel', 10)
        self.completed_pub = self.create_publisher(
            Bool, '/waypoint_follower/completed', 1)
        self.completed = False
        self.completion_publish_count = 0

        if use_gt:
            self._start_gz_subscriber(gz_world)
            self._publish_gz_path_marker()
        else:
            self.create_subscription(
                Odometry, '/mecanum_drive_controller/odom', self._odom_cb, 10)
            self.get_logger().info('Using odometry')

        self.create_timer(0.1, self._control_cb)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        if self.fixed_heading_enabled:
            self.get_logger().info(
                'Fixed-heading stress mode enabled: yaw=%.3f rad. '
                'Route turns become mecanum lateral motion.' % self.fixed_heading_yaw)

    # ------------------------------------------------------------------
    def _start_gz_subscriber(self, gz_world: str):
        """Subscribe to Gazebo dynamic_pose topic via gz.transport (no bridge)."""
        try:
            import gz.transport13 as gz_transport
            import gz.msgs10.pose_v_pb2 as pose_v_pb2

            self._gz_node = gz_transport.Node()
            topic = f'/world/{gz_world}/dynamic_pose/info'

            def _gz_pose_cb(msg):
                for pose in msg.pose:
                    if pose.name == self.robot_model:
                        p = pose.position
                        o = pose.orientation
                        siny = 2.0 * (o.w * o.z + o.x * o.y)
                        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
                        gz_yaw = math.atan2(siny, cosy)
                        with self._lock:
                            self.robot_x   = p.x
                            self.robot_y   = p.y
                            self.robot_yaw = gz_yaw
                            self.pose_ready = True
                        break

            self._gz_node.subscribe(pose_v_pb2.Pose_V, topic, _gz_pose_cb)
            self.get_logger().info(
                f'Subscribed to Gazebo ground truth: {topic}')
        except Exception as e:
            self.get_logger().error(
                f'gz.transport failed: {e} — falling back to odom')
            self.create_subscription(
                Odometry, '/mecanum_drive_controller/odom', self._odom_cb, 10)

    # ------------------------------------------------------------------
    def _publish_gz_path_marker(self):
        """Spawn a static polyline model in Gazebo to visualize the waypoint path."""
        try:
            import gz.msgs10.entity_factory_pb2 as ef_pb2
            import gz.msgs10.boolean_pb2 as bool_pb2

            # Build SDF with thin box segments between consecutive waypoints.
            # polyline would create a filled closed polygon — boxes avoid that.
            segments_sdf = []
            for i in range(len(self.waypoints) - 1):
                x0, y0 = self.waypoints[i]
                x1, y1 = self.waypoints[i + 1]
                mx = (x0 + x1) / 2.0
                my = (y0 + y1) / 2.0
                length = math.hypot(x1 - x0, y1 - y0)
                yaw = math.atan2(y1 - y0, x1 - x0)
                segments_sdf.append(f"""
      <visual name="seg_{i}">
        <pose>{mx} {my} 0.05 0 0 {yaw}</pose>
        <geometry>
          <box><size>{length} 0.03 0.03</size></box>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
          <emissive>0 0.4 0 1</emissive>
        </material>
      </visual>""")

            sdf_xml = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="waypoint_path">
    <static>true</static>
    <link name="link">
{''.join(segments_sdf)}
    </link>
  </model>
</sdf>"""

            req = ef_pb2.EntityFactory()
            req.sdf = sdf_xml
            req.allow_renaming = False

            success, res = self._gz_node.request(
                '/world/factory_world/create',
                req,
                ef_pb2.EntityFactory,
                bool_pb2.Boolean,
                5000)

            if success and res.data:
                self.get_logger().info('Waypoint path spawned in Gazebo')
            else:
                self.get_logger().warn('Failed to spawn waypoint path in Gazebo')
        except Exception as e:
            self.get_logger().warn(f'Gazebo path spawn failed: {e}')

    # ------------------------------------------------------------------
    def _load_waypoints(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    coords = line.split(',')
                    if len(coords) < 2:
                        continue
                    waypoints.append((float(coords[0]), float(coords[1])))
        except Exception as e:
            self.get_logger().error(f'Waypoint load failed: {e}')
        return waypoints

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self.robot_x   = msg.pose.pose.position.x
            self.robot_y   = msg.pose.pose.position.y
            self.robot_yaw = math.atan2(siny, cosy)
            self.pose_ready = True

    # ------------------------------------------------------------------
    def _control_cb(self):
        with self._lock:
            if not self.pose_ready:
                return
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        # Advance past waypoints already within tolerance
        while self.wp_idx < len(self.waypoints):
            wx, wy = self.waypoints[self.wp_idx]
            if math.hypot(wx - rx, wy - ry) < self.tol:
                self.wp_idx += 1
                self.get_logger().info(
                    f'Waypoint {self.wp_idx}/{len(self.waypoints)} reached')
            else:
                break

        if self.wp_idx >= len(self.waypoints):
            self._publish(0.0, 0.0, 0.0)
            if not self.completed:
                self.completed = True
            self.completed_pub.publish(Bool(data=True))
            self.completion_publish_count += 1
            self.get_logger().info('All waypoints reached!', once=True)
            if self.exit_after_completion and self.completion_publish_count >= 10:
                self.get_logger().info('Waypoint follower exiting after completion')
                rclpy.shutdown()
            return

        wp_x, wp_y = self.waypoints[self.wp_idx]
        dx = wp_x - rx
        dy = wp_y - ry

        # World → robot body frame (rotate by -yaw)
        vx = self.kp_l * ( dx * math.cos(ryaw) + dy * math.sin(ryaw))
        vy = self.kp_l * (-dx * math.sin(ryaw) + dy * math.cos(ryaw))

        # Heading: align robot to face the next waypoint
        if self.fixed_heading_enabled:
            target_heading = self.fixed_heading_yaw
        else:
            target_heading = math.atan2(dy, dx)
        heading_err = math.atan2(
            math.sin(target_heading - ryaw),
            math.cos(target_heading - ryaw))
        wz = self.kp_a * heading_err

        vx = max(-self.max_v, min(self.max_v, vx))
        vy = max(-self.max_v, min(self.max_v, vy))
        wz = max(-self.max_w, min(self.max_w, wz))

        self._publish(vx, vy, wz)

    # ------------------------------------------------------------------
    def _publish(self, vx: float, vy: float, wz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
