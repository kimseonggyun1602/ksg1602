#!/usr/bin/env python3
"""Record and evaluate SLAM trajectory against Gazebo ground truth."""

import csv
import json
import math
import os
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryEvaluator(Node):
    """Save paired ground-truth and SLAM poses and continuously update metrics."""

    def __init__(self):
        super().__init__('trajectory_evaluator')

        self.declare_parameter('output_dir', os.path.expanduser(
            '~/ksg_results/current'))
        self.declare_parameter('ground_truth_topic', '/gz_world_poses')
        self.declare_parameter('ground_truth_transform_index', 0)
        self.declare_parameter('estimated_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sample_period_sec', 0.05)
        self.declare_parameter('rpe_interval_sec', 1.0)

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'trajectory.csv')
        self.json_path = os.path.join(
            self.output_dir, 'localization_metrics.json')

        self.gt_index = int(
            self.get_parameter('ground_truth_transform_index').value)
        self.estimated_frame = self.get_parameter('estimated_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        sample_period = float(self.get_parameter('sample_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.gt_queue = deque(maxlen=5000)
        self.rows = []
        self.last_sample_stamp_sec = None

        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec',
            'gt_x_m', 'gt_y_m', 'gt_yaw_rad',
            'slam_x_m', 'slam_y_m', 'slam_yaw_rad',
        ])

        self.gt_path_pub = self.create_publisher(Path, '/evaluation/gt_path', 1)
        self.slam_path_pub = self.create_publisher(
            Path, '/evaluation/slam_path', 1)
        self.gt_path = Path()
        self.gt_path.header.frame_id = self.estimated_frame
        self.slam_path = Path()
        self.slam_path.header.frame_id = self.estimated_frame

        gt_topic = self.get_parameter('ground_truth_topic').value
        self.create_subscription(TFMessage, gt_topic, self.gt_callback, 50)
        self.create_timer(sample_period, self.sample)
        self.create_timer(1.0, self.write_metrics)
        self.get_logger().info(
            f'Evaluating {self.estimated_frame} -> {self.base_frame} '
            f'against {gt_topic}; output={self.output_dir}')

    def gt_callback(self, msg: TFMessage) -> None:
        if len(msg.transforms) <= self.gt_index:
            return
        transform = msg.transforms[self.gt_index]
        stamp = transform.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        p = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        self.gt_queue.append((stamp, p.x, p.y, yaw))

    def sample(self) -> None:
        if not self.gt_queue:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.estimated_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return

        tf_stamp = tf.header.stamp
        tf_stamp_sec = tf_stamp.sec + tf_stamp.nanosec * 1e-9
        if tf_stamp_sec == 0.0:
            tf_stamp = self.gt_queue[-1][0]
            tf_stamp_sec = tf_stamp.sec + tf_stamp.nanosec * 1e-9
        if (
            self.last_sample_stamp_sec is not None
            and tf_stamp_sec <= self.last_sample_stamp_sec + 1e-6
        ):
            return

        stamp, gt_x, gt_y, gt_yaw = min(
            self.gt_queue,
            key=lambda row: abs(
                row[0].sec + row[0].nanosec * 1e-9 - tf_stamp_sec),
        )
        p = tf.transform.translation
        slam_yaw = yaw_from_quaternion(tf.transform.rotation)
        self.last_sample_stamp_sec = tf_stamp_sec
        row = (tf_stamp_sec, gt_x, gt_y, gt_yaw, p.x, p.y, slam_yaw)
        self.rows.append(row)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.append_paths(stamp, gt_x, gt_y, gt_yaw, p.x, p.y, slam_yaw)

    def append_paths(
        self, stamp, gt_x, gt_y, gt_yaw, slam_x, slam_y, slam_yaw
    ) -> None:
        if len(self.rows) < 2:
            return
        metrics, visualization = self.calculate_metrics(include_visualization=True)
        if not metrics:
            return

        gt_map_x, gt_map_y, gt_map_yaw = visualization[-1]
        self.gt_path.header.stamp = stamp
        self.slam_path.header.stamp = stamp
        self.gt_path.poses.append(
            self.make_pose(stamp, gt_map_x, gt_map_y, gt_map_yaw))
        self.slam_path.poses.append(
            self.make_pose(stamp, slam_x, slam_y, slam_yaw))
        self.gt_path_pub.publish(self.gt_path)
        self.slam_path_pub.publish(self.slam_path)

    def make_pose(self, stamp, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.estimated_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def calculate_metrics(self, include_visualization=False):
        if len(self.rows) < 3:
            return ({}, []) if include_visualization else {}

        data = np.asarray(self.rows, dtype=float)
        stamps = data[:, 0]
        gt_xy = data[:, 1:3]
        gt_yaw = data[:, 3]
        slam_xy = data[:, 4:6]
        slam_yaw = data[:, 6]

        slam_center = slam_xy.mean(axis=0)
        gt_center = gt_xy.mean(axis=0)
        centered_slam = slam_xy - slam_center
        centered_gt = gt_xy - gt_center
        u, _, vt = np.linalg.svd(centered_slam.T @ centered_gt)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T @ u.T
        translation = gt_center - rotation @ slam_center
        aligned_xy = (rotation @ slam_xy.T).T + translation
        alignment_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        aligned_yaw = np.asarray([
            normalize_angle(yaw + alignment_yaw) for yaw in slam_yaw])

        position_error = np.linalg.norm(aligned_xy - gt_xy, axis=1)
        yaw_error = np.asarray([
            normalize_angle(a - b) for a, b in zip(aligned_yaw, gt_yaw)])

        rpe_translation = []
        rpe_yaw = []
        interval = float(self.get_parameter('rpe_interval_sec').value)
        for index, stamp in enumerate(stamps):
            target = stamp + interval
            next_index = int(np.searchsorted(stamps, target))
            if next_index >= len(stamps):
                continue
            gt_delta = gt_xy[next_index] - gt_xy[index]
            slam_delta = aligned_xy[next_index] - aligned_xy[index]
            rpe_translation.append(float(np.linalg.norm(slam_delta - gt_delta)))
            gt_delta_yaw = normalize_angle(gt_yaw[next_index] - gt_yaw[index])
            slam_delta_yaw = normalize_angle(
                aligned_yaw[next_index] - aligned_yaw[index])
            rpe_yaw.append(normalize_angle(slam_delta_yaw - gt_delta_yaw))

        metrics = {
            'samples': len(self.rows),
            'duration_sec': float(stamps[-1] - stamps[0]),
            'ate_rmse_m': float(np.sqrt(np.mean(position_error ** 2))),
            'ate_mean_m': float(np.mean(position_error)),
            'yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
            'final_drift_m': float(position_error[-1]),
            'rpe_translation_rmse_m_1s': (
                float(np.sqrt(np.mean(np.square(rpe_translation))))
                if rpe_translation else None
            ),
            'rpe_yaw_rmse_deg_1s': (
                float(np.degrees(np.sqrt(np.mean(np.square(rpe_yaw)))))
                if rpe_yaw else None
            ),
            'alignment': {
                'x_m': float(translation[0]),
                'y_m': float(translation[1]),
                'yaw_deg': float(math.degrees(alignment_yaw)),
            },
        }
        if include_visualization:
            # RViz uses the SLAM map frame. Transform ground truth into that
            # frame while leaving the SLAM trajectory untouched.
            gt_in_map_xy = (rotation.T @ (gt_xy - translation).T).T
            gt_in_map_yaw = np.asarray([
                normalize_angle(yaw - alignment_yaw) for yaw in gt_yaw])
            visualization = np.column_stack((gt_in_map_xy, gt_in_map_yaw))
            return metrics, visualization
        return metrics

    def write_metrics(self) -> None:
        metrics = self.calculate_metrics()
        if not metrics:
            return
        with open(self.json_path, 'w', encoding='utf-8') as output:
            json.dump(metrics, output, indent=2)
        self.get_logger().info(
            'Localization metrics: ATE RMSE=%.3f m, yaw RMSE=%.2f deg, '
            'RPE=%.3f m, final drift=%.3f m, samples=%d' % (
                metrics['ate_rmse_m'],
                metrics['yaw_rmse_deg'],
                metrics['rpe_translation_rmse_m_1s'] or 0.0,
                metrics['final_drift_m'],
                metrics['samples'],
            ))

    def close(self) -> None:
        self.write_metrics()
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryEvaluator()
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
