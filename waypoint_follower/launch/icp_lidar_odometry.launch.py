#!/usr/bin/env python3
"""LaserScan -> RTAB-Map ICP odometry with usable covariance."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    covariance = Node(
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
    return LaunchDescription([icp_odometry, covariance])
