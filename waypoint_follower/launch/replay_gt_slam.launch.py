#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_share = get_package_share_directory('waypoint_follower')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    bag_path = LaunchConfiguration('bag_path')
    playback_rate = LaunchConfiguration('playback_rate')
    use_rviz = LaunchConfiguration('use_rviz')

    gt_pose_to_tf = Node(
        package='waypoint_follower',
        executable='gt_pose_to_tf',
        name='gt_pose_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/gz_world_poses',
            'transform_index': 0,
            'parent_frame': 'gt_odom',
            'child_frame': 'base_footprint',
            'odom_topic': '/gt/odom',
            'path_topic': '/gt/path',
        }],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam_params_file': os.path.join(
                waypoint_share,
                'config',
                'slam_toolbox_gt_mapping.yaml'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_gt_slam',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                slam_toolbox_share,
                'config',
                'slam_toolbox_default.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag_path,
                    '--clock', '100',
                    '--rate', playback_rate,
                    '--disable-keyboard-controls',
                    '--topics',
                    '/scan',
                    '/tf_static',
                    '/gz_world_poses',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.expanduser(
                '~/ajm_bags/factory_lateral_stress_raw')),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        gt_pose_to_tf,
        slam_toolbox,
        rviz,
        bag_play,
    ])
