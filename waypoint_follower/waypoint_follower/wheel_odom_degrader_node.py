#!/usr/bin/env python3
"""Publish a deterministic degraded copy of wheel odometry for repeatable tests."""

import copy
import math
import random

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def set_yaw(q, yaw: float) -> None:
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)


class WheelOdomDegrader(Node):
    """Inject an explainable slip model while preserving the original odometry."""

    def __init__(self):
        super().__init__('wheel_odom_degrader')

        self.declare_parameter('enabled', True)
        self.declare_parameter('input_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('output_topic', '/wheel_odom/degraded')
        self.declare_parameter('linear_x_scale', 0.98)
        self.declare_parameter('linear_y_scale', 0.82)
        self.declare_parameter('yaw_scale', 1.02)
        self.declare_parameter('yaw_bias_rad_per_meter', 0.010)
        self.declare_parameter('translation_noise_std_per_sqrt_meter', 0.010)
        self.declare_parameter('yaw_noise_std_rad_per_sqrt_meter', 0.008)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('slip_events_enabled', True)
        self.declare_parameter('slip_event_start_m', [4.0, 10.0])
        self.declare_parameter('slip_event_end_m', [5.5, 11.5])
        self.declare_parameter('slip_event_linear_x_scale', 0.70)
        self.declare_parameter('slip_event_linear_y_scale', 0.45)
        self.declare_parameter('slip_event_yaw_scale', 1.18)
        self.declare_parameter('slip_event_yaw_bias_rad_per_meter', 0.060)
        self.declare_parameter('slip_event_noise_multiplier', 2.5)
        self.declare_parameter('pose_x_variance_floor', 0.010)
        self.declare_parameter('pose_y_variance_floor', 0.040)
        self.declare_parameter('pose_yaw_variance_floor', 0.020)
        self.declare_parameter('twist_x_variance_floor', 0.010)
        self.declare_parameter('twist_y_variance_floor', 0.040)
        self.declare_parameter('twist_yaw_variance_floor', 0.020)
        self.declare_parameter('slip_event_covariance_multiplier', 4.0)
        self.declare_parameter(
            'diagnostics_topic', '/wheel_odom_degrader/status')
        self.declare_parameter('diagnostics_publish_period_sec', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        diagnostics_topic = self.get_parameter('diagnostics_topic').value
        diagnostics_period = self.get_parameter(
            'diagnostics_publish_period_sec').value

        self.random = random.Random(self.get_parameter('random_seed').value)
        self.last_raw_pose = None
        self.raw_pose = None
        self.degraded_pose = None
        self.total_distance = 0.0
        self.was_enabled = self.is_enabled()
        self.slip_event_active = False
        self.slip_event_samples = 0

        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        diagnostic_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, diagnostics_topic, diagnostic_qos)
        self.create_subscription(Odometry, input_topic, self.odom_callback, 10)
        self.create_timer(diagnostics_period, self.publish_diagnostics)

        self.get_logger().info(
            f'Wheel odometry degrader ready: {input_topic} -> {output_topic}')
        self.log_configuration()

    def is_enabled(self) -> bool:
        return bool(self.get_parameter('enabled').value)

    def is_slip_event_active(self) -> bool:
        if not self.get_parameter('slip_events_enabled').value:
            return False

        starts = self.get_parameter('slip_event_start_m').value
        ends = self.get_parameter('slip_event_end_m').value
        if len(starts) != len(ends):
            self.get_logger().error(
                'slip_event_start_m and slip_event_end_m must have the same length',
                once=True)
            return False
        return any(start <= self.total_distance < end
                   for start, end in zip(starts, ends))

    def odom_callback(self, msg: Odometry) -> None:
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.raw_pose = (raw_x, raw_y, raw_yaw)
        enabled = self.is_enabled()

        if self.last_raw_pose is None:
            self.last_raw_pose = self.raw_pose
            self.degraded_pose = self.raw_pose
            self.publish_odom(msg, self.degraded_pose, enabled)
            return

        last_x, last_y, last_yaw = self.last_raw_pose
        delta_world_x = raw_x - last_x
        delta_world_y = raw_y - last_y
        delta_yaw = normalize_angle(raw_yaw - last_yaw)

        cos_raw = math.cos(last_yaw)
        sin_raw = math.sin(last_yaw)
        delta_body_x = cos_raw * delta_world_x + sin_raw * delta_world_y
        delta_body_y = -sin_raw * delta_world_x + cos_raw * delta_world_y
        distance = math.hypot(delta_body_x, delta_body_y)
        self.total_distance += distance

        if not enabled:
            self.degraded_pose = self.raw_pose
        else:
            if not self.was_enabled:
                # Turning degradation on starts a fresh error accumulation.
                self.degraded_pose = self.last_raw_pose

            self.slip_event_active = self.is_slip_event_active()
            if self.slip_event_active:
                self.slip_event_samples += 1
            event_noise_multiplier = self.get_parameter(
                'slip_event_noise_multiplier').value if self.slip_event_active else 1.0
            distance_sqrt = math.sqrt(max(distance, 0.0))
            translation_noise = self.get_parameter(
                'translation_noise_std_per_sqrt_meter').value * distance_sqrt * event_noise_multiplier
            yaw_noise = self.get_parameter(
                'yaw_noise_std_rad_per_sqrt_meter').value * distance_sqrt * event_noise_multiplier

            x_scale = self.get_parameter('linear_x_scale').value
            y_scale = self.get_parameter('linear_y_scale').value
            yaw_scale = self.get_parameter('yaw_scale').value
            yaw_bias = self.get_parameter('yaw_bias_rad_per_meter').value
            if self.slip_event_active:
                x_scale *= self.get_parameter('slip_event_linear_x_scale').value
                y_scale *= self.get_parameter('slip_event_linear_y_scale').value
                yaw_scale *= self.get_parameter('slip_event_yaw_scale').value
                yaw_bias += self.get_parameter(
                    'slip_event_yaw_bias_rad_per_meter').value

            degraded_body_x = (
                delta_body_x * x_scale
                + self.random.gauss(0.0, translation_noise)
            )
            degraded_body_y = (
                delta_body_y * y_scale
                + self.random.gauss(0.0, translation_noise)
            )
            degraded_delta_yaw = (
                delta_yaw * yaw_scale
                + distance * yaw_bias
                + self.random.gauss(0.0, yaw_noise)
            )

            degraded_x, degraded_y, degraded_yaw = self.degraded_pose
            cos_degraded = math.cos(degraded_yaw)
            sin_degraded = math.sin(degraded_yaw)
            degraded_x += (
                cos_degraded * degraded_body_x - sin_degraded * degraded_body_y)
            degraded_y += (
                sin_degraded * degraded_body_x + cos_degraded * degraded_body_y)
            degraded_yaw = normalize_angle(degraded_yaw + degraded_delta_yaw)
            self.degraded_pose = (degraded_x, degraded_y, degraded_yaw)

        self.last_raw_pose = self.raw_pose
        self.was_enabled = enabled
        self.publish_odom(msg, self.degraded_pose, enabled)

    def publish_odom(self, raw_msg: Odometry, pose, enabled: bool) -> None:
        msg = copy.deepcopy(raw_msg)
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        set_yaw(msg.pose.pose.orientation, pose[2])

        if enabled:
            x_scale = self.get_parameter('linear_x_scale').value
            y_scale = self.get_parameter('linear_y_scale').value
            yaw_scale = self.get_parameter('yaw_scale').value
            if self.slip_event_active:
                x_scale *= self.get_parameter(
                    'slip_event_linear_x_scale').value
                y_scale *= self.get_parameter(
                    'slip_event_linear_y_scale').value
                yaw_scale *= self.get_parameter(
                    'slip_event_yaw_scale').value
            msg.twist.twist.linear.x *= x_scale
            msg.twist.twist.linear.y *= y_scale
            msg.twist.twist.angular.z *= yaw_scale
        # Gazebo publishes zero wheel-odom covariance. Keep clean and degraded
        # runs comparable by reporting the same realistic baseline confidence.
        self.apply_covariance_model(msg)

        self.odom_pub.publish(msg)

    def apply_covariance_model(self, msg: Odometry) -> None:
        multiplier = (
            self.get_parameter('slip_event_covariance_multiplier').value
            if self.slip_event_active else 1.0)
        pose_floors = {
            0: 'pose_x_variance_floor',
            7: 'pose_y_variance_floor',
            35: 'pose_yaw_variance_floor',
        }
        twist_floors = {
            0: 'twist_x_variance_floor',
            7: 'twist_y_variance_floor',
            35: 'twist_yaw_variance_floor',
        }
        for index, parameter in pose_floors.items():
            floor = self.get_parameter(parameter).value * multiplier
            msg.pose.covariance[index] = max(msg.pose.covariance[index], floor)
        for index, parameter in twist_floors.items():
            floor = self.get_parameter(parameter).value * multiplier
            msg.twist.covariance[index] = max(msg.twist.covariance[index], floor)

    def log_configuration(self) -> None:
        self.get_logger().info(
            'enabled=%s, x_scale=%.3f, y_scale=%.3f, yaw_scale=%.3f, '
            'yaw_bias=%.4f rad/m, translation_noise=%.4f m/sqrt(m), '
            'yaw_noise=%.4f rad/sqrt(m), seed=%d' % (
                self.is_enabled(),
                self.get_parameter('linear_x_scale').value,
                self.get_parameter('linear_y_scale').value,
                self.get_parameter('yaw_scale').value,
                self.get_parameter('yaw_bias_rad_per_meter').value,
                self.get_parameter(
                    'translation_noise_std_per_sqrt_meter').value,
                self.get_parameter(
                    'yaw_noise_std_rad_per_sqrt_meter').value,
                self.get_parameter('random_seed').value,
            ))

    def publish_diagnostics(self) -> None:
        if self.raw_pose is None or self.degraded_pose is None:
            return

        raw_x, raw_y, raw_yaw = self.raw_pose
        degraded_x, degraded_y, degraded_yaw = self.degraded_pose
        position_error = math.hypot(degraded_x - raw_x, degraded_y - raw_y)
        yaw_error = normalize_angle(degraded_yaw - raw_yaw)

        values = {
            'enabled': self.is_enabled(),
            'linear_x_scale': self.get_parameter('linear_x_scale').value,
            'linear_y_scale': self.get_parameter('linear_y_scale').value,
            'yaw_scale': self.get_parameter('yaw_scale').value,
            'yaw_bias_rad_per_meter': self.get_parameter(
                'yaw_bias_rad_per_meter').value,
            'translation_noise_std_per_sqrt_meter': self.get_parameter(
                'translation_noise_std_per_sqrt_meter').value,
            'yaw_noise_std_rad_per_sqrt_meter': self.get_parameter(
                'yaw_noise_std_rad_per_sqrt_meter').value,
            'random_seed': self.get_parameter('random_seed').value,
            'slip_events_enabled': self.get_parameter(
                'slip_events_enabled').value,
            'slip_event_start_m': self.get_parameter(
                'slip_event_start_m').value,
            'slip_event_end_m': self.get_parameter('slip_event_end_m').value,
            'slip_event_active': self.slip_event_active,
            'slip_event_samples': self.slip_event_samples,
            'raw_x_m': raw_x,
            'raw_y_m': raw_y,
            'raw_yaw_rad': raw_yaw,
            'degraded_x_m': degraded_x,
            'degraded_y_m': degraded_y,
            'degraded_yaw_rad': degraded_yaw,
            'accumulated_distance_m': self.total_distance,
            'current_position_error_m': position_error,
            'current_yaw_error_rad': yaw_error,
        }

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN if self.is_enabled() else DiagnosticStatus.OK
        status.name = 'wheel_odom_degrader'
        status.message = 'degradation enabled' if self.is_enabled() else 'passthrough'
        status.hardware_id = 'simulation'
        status.values = [
            KeyValue(key=key, value=str(value)) for key, value in values.items()]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomDegrader()
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
