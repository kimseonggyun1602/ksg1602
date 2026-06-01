#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoints_file = os.path.join(
        get_package_share_directory('yahboom_rosmaster_gazebo'),
        'rviz', 'rviz_maps', 'waypoints.txt'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'max_linear_vel', default_value='1.8'),
        DeclareLaunchArgument(
            'max_angular_vel', default_value='1.4'),
        DeclareLaunchArgument(
            'kp_linear', default_value='2.4'),
        DeclareLaunchArgument(
            'kp_angular', default_value='2.3'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.35'),
        DeclareLaunchArgument(
            'exit_after_completion', default_value='false'),
        DeclareLaunchArgument(
            'fixed_heading_enabled', default_value='false'),
        DeclareLaunchArgument(
            'fixed_heading_yaw', default_value='0.0'),

        Node(
            package='waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoints_file': waypoints_file,
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'kp_linear': LaunchConfiguration('kp_linear'),
                'kp_angular': LaunchConfiguration('kp_angular'),
                'exit_after_completion': LaunchConfiguration(
                    'exit_after_completion'),
                'fixed_heading_enabled': LaunchConfiguration(
                    'fixed_heading_enabled'),
                'fixed_heading_yaw': LaunchConfiguration(
                    'fixed_heading_yaw'),
            }],
        ),
    ])
