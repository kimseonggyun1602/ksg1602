# Gazebo + robot_localization + slam_toolbox

## 개요

Gazebo에서 생성한 wheel odometry, IMU, LiDAR scan topic을 ROS 2로 전달합니다.
LiDAR scan으로부터 ICP LiDAR odometry를 생성하고, wheel odometry, IMU, ICP
LiDAR odometry를 `robot_localization`의 EKF에 입력하여 로봇의 위치 TF를
추정합니다.

이 TF와 LiDAR scan을 `slam_toolbox`에 입력하여 실시간 map과 pose를
생성합니다.

전체 실행은 두 단계입니다.

```text
1. Gazebo topics -> robot_localization -> EKF TF
2. /scan + EKF TF -> slam_toolbox -> /map + pose
```

## 처음 한 번만: 설치

Ubuntu 24.04 터미널에서 실행합니다.

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

## 1단계: Gazebo + robot_localization

**터미널 1**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower gazebo_robot_localization.launch.py
```

실행 파일:

```text
waypoint_follower/launch/gazebo_robot_localization.launch.py
```

데이터 흐름:

```text
Gazebo
  /mecanum_drive_controller/odom --+
  /imu/data -----------------------+--> robot_localization EKF
  /scan -> ICP -> /icp/odom -------+        |
                                            +--> /odometry/filtered
                                            +--> TF: odom -> base_footprint
```

EKF 설정 파일:

```text
waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

정상 출력 확인:

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 2단계: slam_toolbox

1단계를 켜둔 상태에서 **터미널 2**를 엽니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 launch waypoint_follower slam_from_robot_localization.launch.py
```

실행 파일:

```text
waypoint_follower/launch/slam_from_robot_localization.launch.py
```

데이터 흐름:

```text
/scan
TF: odom -> base_footprint
TF: base_footprint -> laser_frame
              |
              +--> slam_toolbox
                     +--> /map
                     +--> TF: map -> odom
                     +--> pose: map -> base_footprint
```

정상 출력 확인:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
```

## 로봇 조종

**터미널 3**

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

방향키로 움직이면 RViz에 지도가 생성됩니다.

## 지도 저장

**터미널 4**

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ksg_results/live_slam
ros2 run nav2_map_server map_saver_cli \
  -f ~/ksg_results/live_slam/map
```

저장 위치:

```text
~/ksg_results/live_slam/map.pgm
~/ksg_results/live_slam/map.yaml
```

상세 설치 및 오류 해결은
[상세 가이드](docs/INSTALL_AND_RUN_KO.md)를 참고하세요.
