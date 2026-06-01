#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('waypoint_follower'),
        'config',
        'wheel_odom_degrader.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='waypoint_follower',
            executable='wheel_odom_degrader',
            name='wheel_odom_degrader',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'enabled': LaunchConfiguration('enabled'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
