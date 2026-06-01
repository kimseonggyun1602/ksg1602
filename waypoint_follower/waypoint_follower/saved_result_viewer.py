#!/usr/bin/env python3
"""Publish saved map and trajectory results as RViz-friendly ROS topics."""

import csv
import json
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


FRAME = 'world'


def read_yaml(path):
    with open(path, encoding='utf-8') as source:
        return yaml.safe_load(source)


def resolve_image(yaml_path, image_path):
    if os.path.isabs(image_path):
        return image_path
    return os.path.join(os.path.dirname(yaml_path), image_path)


def occupied_points(yaml_path):
    meta = read_yaml(yaml_path)
    image = np.asarray(
        Image.open(resolve_image(yaml_path, meta['image'])).convert('L'))
    if int(meta.get('negate', 0)):
        occupied = image > int(float(meta['occupied_thresh']) * 255)
    else:
        occupied = image < int((1.0 - float(meta['occupied_thresh'])) * 255)
    rows, cols = np.nonzero(occupied)
    resolution = float(meta['resolution'])
    origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])
    x = origin_x + (cols + 0.5) * resolution
    y = origin_y + (image.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y)), resolution


def transform_xy(points, alignment):
    yaw = math.radians(float(alignment['yaw_deg']))
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[c, -s], [s, c]])
    translation = np.asarray([alignment['x_m'], alignment['y_m']])
    return (rotation @ points.T).T + translation


def read_rows(path):
    with open(path, newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


class SavedResultViewer(Node):
    def __init__(self):
        super().__init__('saved_result_viewer')
        self.declare_parameter('reference_yaml', '')
        self.declare_parameter('gt_csv', '')
        self.declare_parameter('results_dir', '')
        self.declare_parameter('result_id', 'clean')

        reference_yaml = os.path.expanduser(
            self.get_parameter('reference_yaml').value)
        gt_csv = os.path.expanduser(self.get_parameter('gt_csv').value)
        results_dir = os.path.expanduser(
            self.get_parameter('results_dir').value)
        result_id = str(self.get_parameter('result_id').value)
        slam_yaml = os.path.join(results_dir, 'map.yaml')
        slam_csv = os.path.join(results_dir, 'trajectory.csv')
        metrics_json = os.path.join(results_dir, 'localization_metrics.json')
        raw_metrics_json = os.path.join(
            results_dir, 'raw_bag_analysis', 'raw_bag_metrics.json')
        raw_wheel_csv = os.path.join(results_dir, 'raw_wheel_odom.csv')
        degraded_wheel_csv = os.path.join(results_dir, 'degraded_wheel_odom.csv')

        with open(metrics_json, encoding='utf-8') as source:
            alignment = json.load(source)['alignment']
        raw_wheel_alignment = None
        if os.path.isfile(raw_metrics_json):
            with open(raw_metrics_json, encoding='utf-8') as source:
                raw_wheel_alignment = json.load(source)[
                    'raw_wheel_odom_vs_ground_truth']['alignment']

        reference_points, reference_resolution = occupied_points(reference_yaml)
        slam_points, slam_resolution = occupied_points(slam_yaml)
        slam_points = transform_xy(slam_points, alignment)

        gt_rows = read_rows(gt_csv)
        slam_rows = read_rows(slam_csv)
        slam_xy = np.asarray([
            [float(row['slam_x_m']), float(row['slam_y_m'])]
            for row in slam_rows])
        slam_xy = transform_xy(slam_xy, alignment)
        slam_yaw_offset = math.radians(float(alignment['yaw_deg']))

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.reference_pub = self.create_publisher(
            Marker, '/presentation/reference_map_points', qos)
        self.slam_map_pub = self.create_publisher(
            Marker, f'/presentation/{result_id}_slam_map_points', qos)
        self.gt_path_pub = self.create_publisher(
            Path, '/presentation/gt_path', qos)
        self.slam_path_pub = self.create_publisher(
            Path, f'/presentation/{result_id}_slam_path', qos)
        self.raw_wheel_path_pub = self.create_publisher(
            Path, f'/presentation/{result_id}_raw_wheel_path', qos)
        self.degraded_wheel_path_pub = self.create_publisher(
            Path, f'/presentation/{result_id}_degraded_wheel_path', qos)

        self.reference_marker = self.make_points_marker(
            reference_points, reference_resolution, 0.62, 0.62, 0.62,
            'reference_map', 0, 0.00)
        self.slam_map_marker = self.make_points_marker(
            slam_points, slam_resolution, 0.05, 0.48, 0.95,
            f'{result_id}_slam_map', 0, 0.015)
        self.gt_path = self.make_gt_path(gt_rows)
        self.slam_path = self.make_slam_path(
            slam_rows, slam_xy, slam_yaw_offset)
        self.raw_wheel_path = self.make_optional_odom_path(
            raw_wheel_csv, gt_rows, raw_wheel_alignment, 0.10)
        self.degraded_wheel_path = self.make_optional_odom_path(
            degraded_wheel_csv, gt_rows, raw_wheel_alignment, 0.13)

        self.publish_all()
        self.create_timer(1.0, self.publish_all)
        self.get_logger().info(
            'Publishing saved result topics for RViz: reference map, '
            f'{result_id} SLAM map, GT path, raw wheel path, '
            f'degraded wheel path, {result_id} SLAM path')

    def make_points_marker(
        self, points, resolution, red, green, blue, namespace, marker_id, z
    ):
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.025, resolution * 0.82)
        marker.scale.y = max(0.025, resolution * 0.82)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.88
        marker.points = [Point(x=float(x), y=float(y), z=z) for x, y in points]
        return marker

    def make_gt_path(self, rows):
        poses = []
        for row in rows:
            poses.append(self.make_pose(
                float(row['x']), float(row['y']), float(row['yaw_rad']), 0.04))
        return self.make_path(poses)

    def make_slam_path(self, rows, xy, yaw_offset):
        poses = []
        for index, row in enumerate(rows):
            poses.append(self.make_pose(
                xy[index, 0], xy[index, 1],
                float(row['slam_yaw_rad']) + yaw_offset, 0.07))
        return self.make_path(poses)

    def make_optional_odom_path(self, csv_path, gt_rows, alignment, z):
        if not os.path.isfile(csv_path):
            return self.make_path([])
        rows = read_rows(csv_path)
        xy = np.asarray([
            [float(row['x_m']), float(row['y_m'])] for row in rows])
        if alignment is not None:
            xy = transform_xy(xy, alignment)
            yaw_offset = math.radians(float(alignment['yaw_deg']))
        else:
            wheel_initial_yaw = float(rows[0]['yaw_rad'])
            gt_initial_yaw = float(gt_rows[0]['yaw_rad'])
            yaw_offset = gt_initial_yaw - wheel_initial_yaw
            c, s = math.cos(yaw_offset), math.sin(yaw_offset)
            rotation = np.asarray([[c, -s], [s, c]])
            wheel_initial_xy = xy[0]
            gt_initial_xy = np.asarray([
                float(gt_rows[0]['x']), float(gt_rows[0]['y'])])
            xy = (rotation @ (xy - wheel_initial_xy).T).T + gt_initial_xy
        poses = [
            self.make_pose(
                xy[index, 0], xy[index, 1],
                float(row['yaw_rad']) + yaw_offset, z)
            for index, row in enumerate(rows)
        ]
        return self.make_path(poses)

    def make_pose(self, x, y, yaw, z):
        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def make_path(self, poses):
        path = Path()
        path.header.frame_id = FRAME
        path.poses = poses
        return path

    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        self.reference_marker.header.stamp = stamp
        self.slam_map_marker.header.stamp = stamp
        self.gt_path.header.stamp = stamp
        self.slam_path.header.stamp = stamp
        self.raw_wheel_path.header.stamp = stamp
        self.degraded_wheel_path.header.stamp = stamp
        for pose in self.gt_path.poses:
            pose.header.stamp = stamp
        for pose in self.slam_path.poses:
            pose.header.stamp = stamp
        for pose in self.raw_wheel_path.poses:
            pose.header.stamp = stamp
        for pose in self.degraded_wheel_path.poses:
            pose.header.stamp = stamp
        self.reference_pub.publish(self.reference_marker)
        self.slam_map_pub.publish(self.slam_map_marker)
        self.gt_path_pub.publish(self.gt_path)
        self.slam_path_pub.publish(self.slam_path)
        self.raw_wheel_path_pub.publish(self.raw_wheel_path)
        self.degraded_wheel_path_pub.publish(self.degraded_wheel_path)


def main(args=None):
    rclpy.init(args=args)
    node = SavedResultViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
