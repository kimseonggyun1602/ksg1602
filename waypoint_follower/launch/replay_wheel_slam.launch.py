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
    default_bag = os.path.expanduser(
        '~/ajm_bags/factory_raw_20260531_185946')

    bag_path = LaunchConfiguration('bag_path')
    playback_rate = LaunchConfiguration('playback_rate')
    use_rviz = LaunchConfiguration('use_rviz')
    degradation_enabled = LaunchConfiguration('degradation_enabled')
    results_dir = LaunchConfiguration('results_dir')

    wheel_odom_degrader = Node(
        package='waypoint_follower',
        executable='wheel_odom_degrader',
        name='wheel_odom_degrader',
        output='screen',
        parameters=[
            os.path.join(
                waypoint_share, 'config', 'wheel_odom_degrader.yaml'),
            {
                'enabled': degradation_enabled,
                'use_sim_time': True,
            },
        ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    odom_to_tf = Node(
        package='waypoint_follower',
        executable='odom_to_tf',
        name='wheel_odom_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/wheel_odom/degraded',
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
        }],
    )

    trajectory_evaluator = Node(
        package='waypoint_follower',
        executable='trajectory_evaluator',
        name='trajectory_evaluator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': results_dir,
        }],
    )

    wheel_path_recorder = Node(
        package='waypoint_follower',
        executable='wheel_path_recorder',
        name='wheel_path_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'results_dir': results_dir,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_wheel_slam',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                slam_toolbox_share, 'config', 'slam_toolbox_default.rviz'),
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
                    '/mecanum_drive_controller/odom',
                    '/tf_static',
                    '/gz_world_poses',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value=default_bag),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('degradation_enabled', default_value='true'),
        DeclareLaunchArgument(
            'results_dir',
            default_value=os.path.expanduser(
                '~/ksg_results/strong_wheel_only')),
        wheel_odom_degrader,
        odom_to_tf,
        trajectory_evaluator,
        wheel_path_recorder,
        slam_toolbox,
        rviz,
        bag_play,
    ])
