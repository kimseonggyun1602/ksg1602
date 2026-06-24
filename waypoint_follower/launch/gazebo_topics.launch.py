#!/usr/bin/env python3
"""Start Gazebo and publish wheel odom, IMU, LaserScan, and static TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_share = get_package_share_directory('yahboom_rosmaster_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 'yahboom_rosmaster.gazebo.launch.py')),
        launch_arguments={
            'enable_odom_tf': 'false',
            'headless': 'False',
            'load_controllers': 'true',
            'world_file': 'factory_map_10m.world',
            'gz_world_name': 'factory_world',
            'use_gz_pose_tf': 'false',
            'use_rviz': 'false',
            'use_robot_state_pub': 'true',
            'use_sim_time': 'true',
            'x': '-4.45',
            'y': '4.45',
            'z': '0.10',
            'yaw': '0.0',
        }.items(),
    )
    return LaunchDescription([gazebo])
