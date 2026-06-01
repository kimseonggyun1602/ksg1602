# ROS 2 Jazzy Mecanum SLAM Reproduction Guide

This guide reproduces the Gazebo simulation pipeline used for the ROSMASTER X3
mecanum-wheel robot on Ubuntu 24.04 and ROS 2 Jazzy.

The stable baseline is:

```text
Gazebo wheel odom + IMU
  -> robot_localization EKF
  -> TF: odom -> base_footprint

/scan + /tf_static + EKF TF
  -> slam_toolbox
  -> /map
  -> TF: map -> odom
```

ICP LiDAR odometry is an optional experiment:

```text
/scan
  -> RTAB-Map ICP odometry
  -> /icp/odom_raw
  -> covariance scaler
  -> /icp/odom
  -> robot_localization EKF input
```

Use the baseline first. Enable ICP only when comparing behavior under wheel slip
or odometry degradation. ICP may reduce accuracy when the wheel odometry is
already nearly ideal.

## 1. Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic via `ros_gz`

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-nav2-map-server
```

Initialize `rosdep` once:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 2. Workspace Setup

Clone the modified project repository into a clean workspace:

```bash
mkdir -p ~/ksg_ws/src
cd ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git yahboom_rosmaster
```

Do not clone only the original upstream repository, because this repository
includes additional Jazzy simulation, localization, SLAM, and evaluation files.

### Build the workspace

Build the workspace:

```bash
cd ~/ksg_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --allow-overriding mecanum_drive_controller
source install/setup.bash
```

## 3. Important Project Files

### Gazebo simulation

```text
yahboom_rosmaster_gazebo/launch/yahboom_rosmaster.gazebo.launch.py
yahboom_rosmaster_gazebo/worlds/factory_map_10m.world
yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml
```

The clock bridge must contain:

```yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/world/factory_world/clock"
```

### EKF localization

```text
waypoint_follower/config/ekf_wheel_imu_basic.yaml
waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

### Optional ICP odometry

```text
waypoint_follower/waypoint_follower/odom_covariance_scaler_node.py
```

### Evaluation and control tools

```text
waypoint_follower/waypoint_follower/keyboard_teleop_node.py
waypoint_follower/waypoint_follower/trajectory_evaluator_node.py
waypoint_follower/waypoint_follower/map_evaluator.py
```

## 4. Stop Existing Processes

Run before starting a fresh experiment:

```bash
pkill -f "gz sim" || true
pkill -f "rviz2" || true
pkill -f "ekf_node" || true
pkill -f "icp_odometry" || true
pkill -f "odom_covariance_scaler" || true
pkill -f "async_slam_toolbox_node" || true
pkill -f "keyboard_teleop" || true
pkill -f "trajectory_evaluator" || true
```

## 5. Terminal 1: Start Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py \
  enable_odom_tf:=false \
  headless:=False \
  load_controllers:=true \
  world_file:=factory_map_10m.world \
  gz_world_name:=factory_world \
  use_gz_pose_tf:=false \
  use_rviz:=false \
  use_robot_state_pub:=true \
  use_sim_time:=true \
  x:=-4.45 \
  y:=4.45 \
  z:=0.10 \
  yaw:=0.0
```

Wait about 30 seconds for controller loading. Verify in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 topic hz /clock
ros2 control list_controllers
ros2 topic hz /scan
```

Expected controllers:

```text
joint_state_broadcaster     active
mecanum_drive_controller    active
```

If controllers remain `unconfigured`, activate them manually:

```bash
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state mecanum_drive_controller active
```

## 6A. Stable Baseline: Wheel Odom + IMU EKF

Skip this section when running the ICP experiment in section 6B.

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_basic.yaml
```

Verify:

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 6B. Optional Experiment: Wheel Odom + IMU + ICP EKF

Do not run the baseline EKF from section 6A at the same time.

Terminal 2, start ICP LiDAR odometry:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run rtabmap_odom icp_odometry \
  --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_footprint \
  -p odom_frame_id:=odom \
  -p publish_tf:=false \
  -p wait_for_transform:=0.2 \
  -r odom:=/icp/odom_raw
```

Terminal 3, apply conservative ICP covariance floors:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower odom_covariance_scaler \
  --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/icp/odom_raw \
  -p output_topic:=/icp/odom
```

Verify:

```bash
ros2 topic echo /icp/odom --once
```

Terminal 4, start the EKF:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

Verify:

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 7. Start slam_toolbox

Run after either section 6A or 6B.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true
```

Verify:

```bash
ros2 topic info /map
ros2 run tf2_ros tf2_echo map base_footprint
```

Expected data flow:

```text
/scan + TF: odom -> base_footprint + /tf_static
  -> slam_toolbox
  -> /map
  -> TF: map -> odom
```

## 8. Publish Ground-Truth and SLAM Paths

Use a new output directory for each run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower trajectory_evaluator \
  --ros-args \
  -p use_sim_time:=true \
  -p ground_truth_topic:=/gz_world_poses \
  -p estimated_frame:=map \
  -p base_frame:=base_footprint \
  -p output_dir:=$HOME/ksg_results/live_slam_wheel_imu_icp
```

Published paths:

```text
/evaluation/gt_path
/evaluation/slam_path
```

## 9. Start RViz

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

Configure RViz:

```text
Global Options
  Fixed Frame: map

Add -> Map
  Topic: /map

Add -> Path
  Topic: /evaluation/gt_path
  Color: green

Add -> Path
  Topic: /evaluation/slam_path
  Color: red

Optional:
  Add -> TF
  Add -> RobotModel
```

Interpretation:

```text
Gray occupancy grid: slam_toolbox map
Green path: Gazebo physical ground truth from /gz_world_poses
Red path: final SLAM pose map -> base_footprint
```

## 10. Keyboard Control

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

Use arrow keys or:

```text
i: forward
,: backward
j: rotate left
l: rotate right
```

## 11. Save and Evaluate the Map

Save the SLAM map:

```bash
mkdir -p ~/ksg_results/live_slam_compare

ros2 run nav2_map_server map_saver_cli \
  -f ~/ksg_results/live_slam_compare/map
```

Evaluate against the Gazebo reference map:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower map_evaluator \
  --reference-yaml \
  ~/ksg_ws/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/rviz/rviz_maps/my_factory_map.yaml \
  --slam-yaml \
  ~/ksg_results/live_slam_compare/map.yaml \
  --output-dir \
  ~/ksg_results/live_slam_compare/evaluation
```

Output files:

```text
~/ksg_results/live_slam_compare/map.pgm
~/ksg_results/live_slam_compare/map.yaml
~/ksg_results/live_slam_compare/evaluation/mapping_metrics.json
~/ksg_results/live_slam_compare/evaluation/mapping_overlay.png
```

Important metrics:

```text
wall_rmse_m
wall_precision_15cm
wall_recall_15cm
occupied_cell_iou_exact
```

## 12. Topics and TF Summary

```text
Gazebo outputs:
  /mecanum_drive_controller/odom
  /imu/data
  /scan
  /tf_static
  /gz_world_poses

Optional ICP:
  /icp/odom_raw
  /icp/odom

EKF outputs:
  /odometry/filtered
  TF: odom -> base_footprint

slam_toolbox outputs:
  /map
  TF: map -> odom

Final SLAM pose:
  TF: map -> base_footprint
```


