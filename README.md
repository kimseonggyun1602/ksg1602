# Gazebo + robot_localization + slam_toolbox

## 개요

Gazebo에서 생성한 wheel odometry, IMU, LiDAR scan topic을 ROS 2로
전달합니다. LiDAR scan으로 ICP LiDAR odometry를 생성하고, wheel odometry,
IMU, ICP LiDAR odometry를 `robot_localization`의 EKF에 입력하여 로봇의 위치
TF를 추정합니다.

이 TF와 LiDAR scan을 `slam_toolbox`에 입력하여 실시간 map과 pose를
생성합니다.

## 설치 및 빌드

Ubuntu 24.04와 ROS 2 Jazzy가 설치된 상태를 기준으로 합니다.

### 1. Gazebo와 ROS 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server
```

### 2. GitHub 저장소 다운로드

```bash
mkdir -p ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git \
  ~/ksg_ws/src/yahboom_rosmaster
```

### 3. Workspace 의존성 설치 및 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ksg_ws

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --allow-overriding mecanum_drive_controller

source ~/ksg_ws/install/setup.bash
```

빌드가 완료되면 아래 `터미널 1`부터 순서대로 실행합니다.

## 실행 순서

### 터미널 1: Gazebo 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower gazebo_topics.launch.py
```

출력 topic:

```text
/mecanum_drive_controller/odom
/imu/data
/scan
/tf_static
/gz_world_poses
```

Gazebo가 완전히 열린 뒤 약 30초 기다립니다.

### 터미널 2: ICP LiDAR odometry 생성

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower icp_lidar_odometry.launch.py
```

```text
입력: /scan
출력: /icp/odom
```

### 터미널 3: Wheel + IMU + ICP EKF 융합

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower wheel_imu_icp_ekf.launch.py
```

```text
입력: /mecanum_drive_controller/odom, /imu/data, /icp/odom
출력: /odometry/filtered, TF odom -> base_footprint
설정: waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

### 터미널 4: slam_toolbox 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower slam_from_robot_localization.launch.py
```

```text
입력: /scan, TF odom -> base_footprint, /tf_static
출력: /map, TF map -> odom, 최종 pose map -> base_footprint
```

### 터미널 5: Gazebo 로봇 조종

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

조종 노드를 실행한 뒤 아직 움직이지 말고 터미널 6을 먼저 실행합니다.

### 터미널 6: RViz에서 SLAM 결과 비교

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower slam_gt_rviz.launch.py
```

RViz 표시:

```text
회색 지도: slam_toolbox가 생성한 /map
초록 선:   Gazebo 실제 로봇 경로 (/gz_world_poses)
빨간 선:   SLAM 추정 경로 (TF map -> base_footprint)
```

RViz가 열린 다음 터미널 5에 포커스를 두고 방향키로 로봇을 움직입니다.

## 출력 확인

```bash
ros2 topic echo /icp/odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
```

## 지도 저장

```bash
mkdir -p ~/ksg_results/live_slam
ros2 run nav2_map_server map_saver_cli \
  -f ~/ksg_results/live_slam/map
```

자세한 설치 및 오류 해결은
[상세 가이드](docs/INSTALL_AND_RUN_KO.md)를 참고하세요.
