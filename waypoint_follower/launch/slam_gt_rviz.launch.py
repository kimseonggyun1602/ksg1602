#!/usr/bin/env python3
"""Display Gazebo ground truth and SLAM trajectories in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_share = get_package_share_directory('waypoint_follower')

    evaluator = Node(
        package='waypoint_follower',
        executable='trajectory_evaluator',
        name='slam_trajectory_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ground_truth_topic': '/gz_world_poses',
            'estimated_frame': 'map',
            'base_frame': 'base_footprint',
            'sample_period_sec': 0.05,
            'output_dir': os.path.expanduser(
                '~/ksg_results/live_slam_trajectory'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_comparison_rviz',
        output='screen',
        arguments=['-d', os.path.join(
            waypoint_share, 'rviz', 'slam_gt_comparison.rviz')],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([evaluator, rviz])
