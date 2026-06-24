# Gazebo + robot_localization + slam_toolbox

## 개요

Gazebo에서 생성한 wheel odometry, IMU, LiDAR scan topic을 ROS 2로 전달합니다.
LiDAR scan으로 ICP LiDAR odometry를 생성하고, 세 센서 정보를
`robot_localization`의 EKF에 입력하여 로봇의 위치 TF를 추정합니다.

이 TF와 LiDAR scan을 `slam_toolbox`에 입력하여 실시간 map과 pose를
생성합니다.

```text
Gazebo Wheel odom -----+
Gazebo IMU ------------+--> robot_localization EKF
LiDAR scan -> ICP odom -+        -> TF: odom -> base_footprint
                                      |
LiDAR scan ---------------------------+--> slam_toolbox
                                            -> /map
                                            -> pose: map -> base_footprint
```

## 처음 한 번만: 설치 및 빌드

```bash
curl -fsSL \
  https://raw.githubusercontent.com/kimseonggyun1602/ksg1602/main/scripts/install_ubuntu_24_04_ros2_jazzy.sh \
  -o /tmp/install_ksg_ros2.sh
bash /tmp/install_ksg_ros2.sh

mkdir -p ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git \
  ~/ksg_ws/src/yahboom_rosmaster
bash ~/ksg_ws/src/yahboom_rosmaster/scripts/build_ksg_ws.sh
```

## 실행

### 터미널 1: Gazebo + ICP + EKF

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower gazebo_robot_localization.launch.py
```

Gazebo가 열리고 센서와 controller가 준비될 때까지 약 30초 기다립니다.

### 터미널 2: slam_toolbox + RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower slam_from_robot_localization.launch.py
```

### 터미널 3: 조종

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

방향키로 로봇을 움직이면 RViz에 실시간 지도가 생성됩니다.

## 실행 결과 확인

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
```

```text
/odometry/filtered           EKF 위치 추정 결과
odom -> base_footprint       EKF가 생성한 TF
/map                         slam_toolbox가 생성한 지도
map -> base_footprint        최종 SLAM pose
```

## 실행 파일

```text
waypoint_follower/launch/gazebo_robot_localization.launch.py
  Gazebo + ICP odometry + robot_localization 실행

waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
  EKF 입력 센서 및 상태 설정

waypoint_follower/launch/slam_from_robot_localization.launch.py
  slam_toolbox + RViz 실행
```

자세한 설치 및 오류 해결은
[상세 가이드](docs/INSTALL_AND_RUN_KO.md)를 참고하세요.
