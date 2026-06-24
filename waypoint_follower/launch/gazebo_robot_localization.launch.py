#!/usr/bin/env python3
"""Gazebo topics -> Wheel+IMU+ICP robot_localization EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_share = get_package_share_directory('waypoint_follower')
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

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'wait_for_transform': 0.3,
            'approx_sync': False,
        }],
        remappings=[
            ('odom', '/icp/odom_raw'),
            ('scan_cloud', '/unused_scan_cloud'),
        ],
    )

    icp_covariance = Node(
        package='waypoint_follower',
        executable='odom_covariance_scaler',
        name='icp_odom_covariance',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/icp/odom_raw',
            'output_topic': '/icp/odom',
            'pose_xy_variance_floor': 0.02,
            'pose_yaw_variance_floor': 0.02,
            'twist_xy_variance_floor': 0.04,
            'twist_yaw_variance_floor': 0.03,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            waypoint_share, 'config', 'ekf_wheel_imu_icp_basic.yaml')],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=2.0, actions=[icp_odometry, icp_covariance, ekf]),
    ])
