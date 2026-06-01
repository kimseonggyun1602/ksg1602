#!/usr/bin/env python3
"""Render reference and aligned SLAM maps with trajectories in world coordinates."""

import argparse
import csv
import json
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def read_yaml(path):
    with open(path, encoding='utf-8') as source:
        return yaml.safe_load(source)


def resolve_image(yaml_path, image_path):
    if os.path.isabs(image_path):
        return image_path
    return os.path.join(os.path.dirname(yaml_path), image_path)


def occupied_points(yaml_path):
    meta = read_yaml(yaml_path)
    image = np.asarray(Image.open(resolve_image(yaml_path, meta['image'])).convert('L'))
    if int(meta.get('negate', 0)):
        occupied = image > int(float(meta['occupied_thresh']) * 255)
    else:
        occupied = image < int((1.0 - float(meta['occupied_thresh'])) * 255)
    rows, cols = np.nonzero(occupied)
    resolution = float(meta['resolution'])
    origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])
    x = origin_x + (cols + 0.5) * resolution
    y = origin_y + (image.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y))


def read_csv_columns(path, names):
    with open(path, newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))
    return {
        name: np.asarray([float(row[name]) for row in rows])
        for name in names
    }


def transform_xy(points, alignment):
    yaw = math.radians(float(alignment['yaw_deg']))
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[c, -s], [s, c]])
    translation = np.asarray([alignment['x_m'], alignment['y_m']])
    return (rotation @ points.T).T + translation


def draw_map(axis, points, color, label, size=3.0, alpha=1.0):
    axis.scatter(points[:, 0], points[:, 1], s=size, c=color, alpha=alpha,
                 marker='s', linewidths=0, label=label)


def draw_gt(axis, gt):
    axis.plot(gt['x'], gt['y'], color='#d62728', linewidth=2.2,
              label='Gazebo ground truth trajectory', zorder=5)
    axis.scatter(gt['x'][0], gt['y'][0], color='#1f77b4', s=36,
                 label='Start', zorder=6)
    axis.scatter(gt['x'][-1], gt['y'][-1], color='#111111', s=36,
                 label='End', zorder=6)


def style(axis, title):
    axis.set_title(title)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True, linewidth=0.4, alpha=0.25)
    axis.legend(loc='best', fontsize=8)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference-yaml', required=True)
    parser.add_argument('--gt-csv', required=True)
    parser.add_argument('--results-dir', required=True)
    parser.add_argument('--output')
    args = parser.parse_args()

    reference_yaml = os.path.abspath(os.path.expanduser(args.reference_yaml))
    gt_csv = os.path.abspath(os.path.expanduser(args.gt_csv))
    results_dir = os.path.abspath(os.path.expanduser(args.results_dir))
    slam_yaml = os.path.join(results_dir, 'map.yaml')
    trajectory_csv = os.path.join(results_dir, 'trajectory.csv')
    metrics_json = os.path.join(results_dir, 'localization_metrics.json')
    output = (
        os.path.abspath(os.path.expanduser(args.output))
        if args.output else os.path.join(results_dir, 'map_trajectory_comparison.png'))

    reference_map = occupied_points(reference_yaml)
    slam_map = occupied_points(slam_yaml)
    gt_data = read_csv_columns(gt_csv, ['x', 'y'])
    gt = {'x': gt_data['x'], 'y': gt_data['y']}
    slam_path = read_csv_columns(trajectory_csv, ['slam_x_m', 'slam_y_m'])
    with open(metrics_json, encoding='utf-8') as source:
        alignment = json.load(source)['alignment']

    aligned_slam_map = transform_xy(slam_map, alignment)
    aligned_slam_path = transform_xy(
        np.column_stack((slam_path['slam_x_m'], slam_path['slam_y_m'])),
        alignment)

    figure, axes = plt.subplots(1, 2, figsize=(15, 7.2), dpi=170,
                                sharex=True, sharey=True)

    draw_map(axes[0], reference_map, '#222222', 'Gazebo reference map', 4.0)
    draw_gt(axes[0], gt)
    style(axes[0], 'Ground truth map and trajectory')

    draw_map(axes[1], reference_map, '#d9d9d9', 'Gazebo reference map', 3.4)
    draw_map(axes[1], aligned_slam_map, '#1f77b4', 'Clean SLAM map', 3.0, 0.9)
    draw_gt(axes[1], gt)
    axes[1].plot(aligned_slam_path[:, 0], aligned_slam_path[:, 1],
                 color='#2ca02c', linewidth=2.2, marker='o', markersize=3.0,
                 label='Clean SLAM pose', zorder=6)
    style(axes[1], 'Clean lateral wheel-only SLAM result')

    figure.suptitle('Reference vs. clean lateral wheel-only SLAM', fontsize=15)
    figure.tight_layout()
    figure.savefig(output)
    print(output)


if __name__ == '__main__':
    main()
