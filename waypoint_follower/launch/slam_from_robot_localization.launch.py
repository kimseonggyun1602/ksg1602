#!/usr/bin/env python3
"""EKF TF + LaserScan -> slam_toolbox map and pose."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('slam_toolbox')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            slam_share, 'config', 'slam_toolbox_default.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        slam,
        TimerAction(period=3.0, actions=[rviz]),
    ])
