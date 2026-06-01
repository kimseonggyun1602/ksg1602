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
        '~/ajm_bags/factory_lateral_stress_raw')

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
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('odom', '/icp/odom_raw'),
        ],
    )

    icp_covariance_scaler = Node(
        package='waypoint_follower',
        executable='odom_covariance_scaler',
        name='icp_odom_covariance_scaler',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/icp/odom_raw',
            'output_topic': '/icp/odom',
            'pose_xy_variance_floor': 0.020,
            'pose_yaw_variance_floor': 0.015,
            'twist_xy_variance_floor': 0.040,
            'twist_yaw_variance_floor': 0.020,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                waypoint_share, 'config', 'ekf_wheel_imu_icp.yaml'),
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered_wheel_imu_icp'),
        ],
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

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_wheel_imu_icp_slam',
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
                    '/imu/data',
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
                '~/ksg_results/strong_lateral_wheel_imu_icp')),
        wheel_odom_degrader,
        icp_covariance_scaler,
        icp_odometry,
        ekf,
        trajectory_evaluator,
        slam_toolbox,
        rviz,
        bag_play,
    ])
