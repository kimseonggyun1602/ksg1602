#!/usr/bin/env python3
"""Plot GT, raw wheel odometry, and SLAM pose trajectories in one figure."""

import argparse
import csv
import json
import math
import os

import matplotlib.pyplot as plt
import numpy as np


def read_columns(path, names):
    with open(path, newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))
    return {
        name: np.asarray([float(row[name]) for row in rows])
        for name in names
    }


def transform_slam_to_gt(x, y, alignment):
    yaw = math.radians(alignment['yaw_deg'])
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        c * x - s * y + alignment['x_m'],
        s * x + c * y + alignment['y_m'],
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir', required=True)
    args = parser.parse_args()
    results_dir = os.path.abspath(os.path.expanduser(args.results_dir))

    slam_csv = os.path.join(results_dir, 'trajectory.csv')
    raw_csv = os.path.join(
        results_dir, 'raw_bag_analysis', 'raw_wheel_odom_vs_gt.csv')
    metrics_json = os.path.join(results_dir, 'localization_metrics.json')
    output_png = os.path.join(results_dir, 'trajectory_plot.png')

    raw = read_columns(raw_csv, [
        'gt_x_m', 'gt_y_m', 'aligned_wheel_odom_x_m',
        'aligned_wheel_odom_y_m'])
    slam = read_columns(slam_csv, [
        'gt_x_m', 'gt_y_m', 'slam_x_m', 'slam_y_m'])
    with open(metrics_json, encoding='utf-8') as source:
        metrics = json.load(source)

    slam_gt_x, slam_gt_y = transform_slam_to_gt(
        slam['slam_x_m'], slam['slam_y_m'], metrics['alignment'])

    figure, axis = plt.subplots(figsize=(8, 8), dpi=160)
    axis.plot(raw['gt_x_m'], raw['gt_y_m'], color='black', linewidth=2.4,
              label='Gazebo ground truth')
    axis.plot(raw['aligned_wheel_odom_x_m'], raw['aligned_wheel_odom_y_m'],
              color='#f28e2b', linewidth=1.8, linestyle='--',
              label='Raw wheel odometry')
    axis.plot(slam_gt_x, slam_gt_y, color='#2ca02c', linewidth=2.0,
              marker='o', markersize=2.8, label='SLAM pose')

    axis.scatter(raw['gt_x_m'][0], raw['gt_y_m'][0], color='#1f77b4',
                 s=50, zorder=5, label='Start')
    axis.scatter(raw['gt_x_m'][-1], raw['gt_y_m'][-1], color='#d62728',
                 s=50, zorder=5, label='End')
    axis.set_title('Clean lateral wheel-only trajectory comparison')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.axis('equal')
    axis.grid(True, linewidth=0.5, alpha=0.4)
    axis.legend(loc='best')
    figure.tight_layout()
    figure.savefig(output_png)
    print(output_png)


if __name__ == '__main__':
    main()
