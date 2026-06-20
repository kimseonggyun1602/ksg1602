#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_share = get_package_share_directory('waypoint_follower')
    gazebo_share = get_package_share_directory('yahboom_rosmaster_gazebo')

    default_reference = os.path.join(
        gazebo_share, 'rviz', 'rviz_maps', 'my_factory_map.yaml')
    default_gt = os.path.expanduser(
        '~/ksg_results/gt_paths/factory_lateral_stress_gt_world_clean.csv')
    default_results = os.path.expanduser(
        '~/ksg_results/slip_wheel_imu_icp')

    return LaunchDescription([
        DeclareLaunchArgument(
            'reference_yaml', default_value=default_reference),
        DeclareLaunchArgument('gt_csv', default_value=default_gt),
        DeclareLaunchArgument('results_dir', default_value=default_results),
        Node(
            package='waypoint_follower',
            executable='saved_result_viewer',
            name='saved_slip_result_viewer',
            output='screen',
            parameters=[{
                'reference_yaml': LaunchConfiguration('reference_yaml'),
                'gt_csv': LaunchConfiguration('gt_csv'),
                'results_dir': LaunchConfiguration('results_dir'),
                'result_id': 'slip',
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_saved_slip_result_viewer',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    waypoint_share, 'rviz', 'slip_result_viewer.rviz'),
            ],
        ),
    ])
