from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajm',
    maintainer_email='ajm1122383@gmail.com',
    description='Waypoint follower for mecanum-wheel robot',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'waypoint_follower = waypoint_follower.waypoint_follower_node:main',
            'gz_pose_tf_publisher = waypoint_follower.gz_pose_tf_publisher:main',
            'keyboard_teleop = waypoint_follower.keyboard_teleop_node:main',
            'map_evaluator = waypoint_follower.map_evaluator:main',
            'map_trajectory_comparison = waypoint_follower.map_trajectory_comparison:main',
            'raw_bag_evaluator = waypoint_follower.raw_bag_evaluator:main',
            'odom_to_tf = waypoint_follower.odom_to_tf_node:main',
            'saved_result_viewer = waypoint_follower.saved_result_viewer:main',
            'trajectory_evaluator = waypoint_follower.trajectory_evaluator_node:main',
            'trajectory_plotter = waypoint_follower.trajectory_plotter:main',
            'wheel_odom_degrader = waypoint_follower.wheel_odom_degrader_node:main',
            'odom_covariance_scaler = waypoint_follower.odom_covariance_scaler_node:main',
            'wheel_path_recorder = waypoint_follower.wheel_path_recorder_node:main',
        ],
    },
)
