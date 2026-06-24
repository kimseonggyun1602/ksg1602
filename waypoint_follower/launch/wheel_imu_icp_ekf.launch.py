#!/usr/bin/env python3
"""Wheel odom + IMU + ICP odom -> robot_localization EKF TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_share = get_package_share_directory('waypoint_follower')
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            waypoint_share, 'config', 'ekf_wheel_imu_icp_basic.yaml')],
    )
    return LaunchDescription([ekf])
