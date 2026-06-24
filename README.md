# Gazebo Wheel Odom + IMU + ICP EKF + slam_toolbox

Ubuntu 24.04와 ROS 2 Jazzy에서 ROSMASTER X3 메카넘 로봇을 Gazebo로
시뮬레이션하고, `robot_localization`과 `slam_toolbox`로 위치 추정 및 2D SLAM을
재현하는 저장소입니다.

```text
Gazebo
  /mecanum_drive_controller/odom --+
  /imu/data -----------------------+--> robot_localization EKF
  /scan -> RTAB-Map ICP -> /icp/odom --+   -> /odometry/filtered
                                           -> TF: odom -> base_footprint

  /scan + /tf_static + EKF TF
        -> slam_toolbox
        -> /map, /pose, TF: map -> odom
```

## 처음 시작하기

ROS 2가 전혀 설치되지 않은 Ubuntu 24.04부터 시작하는 전체 절차는 아래 문서에
정리되어 있습니다.

**[Ubuntu 초기 상태부터 SLAM 실행까지 A-to-Z 가이드](docs/INSTALL_AND_RUN_KO.md)**

설치는 다음 세 단계로 시작할 수 있습니다.

```bash
# 1. ROS 2 Jazzy와 필수 패키지 설치
curl -fsSL \
  https://raw.githubusercontent.com/kimseonggyun1602/ksg1602/main/scripts/install_ubuntu_24_04_ros2_jazzy.sh \
  -o /tmp/install_ksg_ros2.sh
bash /tmp/install_ksg_ros2.sh

# 2. 저장소 다운로드
mkdir -p ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git \
  ~/ksg_ws/src/yahboom_rosmaster

# 3. 워크스페이스 빌드
bash ~/ksg_ws/src/yahboom_rosmaster/scripts/build_ksg_ws.sh
```

새 터미널마다 다음 두 줄을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
```

## 주요 입출력

| 단계 | 입력 | 출력 |
|---|---|---|
| Gazebo | `/mecanum_drive_controller/cmd_vel` | Wheel odom, IMU, LaserScan, static TF |
| ICP odometry | `/scan` | `/icp/odom_raw` |
| EKF | Wheel odom + IMU + `/icp/odom` | `/odometry/filtered`, `odom -> base_footprint` |
| slam_toolbox | `/scan` + TF | `/map`, `/pose`, `map -> odom` |

## 주요 설정 파일

```text
waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
waypoint_follower/waypoint_follower/odom_covariance_scaler_node.py
yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml
yahboom_rosmaster_gazebo/worlds/factory_map_10m.world
```

## 주의사항

- Gazebo 실행 시 `enable_odom_tf:=false`를 사용합니다. 최종
  `odom -> base_footprint` TF는 EKF 하나만 발행해야 합니다.
- Wheel+IMU 기준 실험과 Wheel+IMU+ICP 실험의 EKF를 동시에 실행하지 마세요.
- 이상적인 Gazebo wheel odom에서는 ICP 추가가 반드시 성능을 높이지 않습니다.
  ICP 효과는 wheel slip 또는 encoder 오차가 있는 조건에서 비교하는 것이 적절합니다.
- 저장소의 라이선스와 각 외부 ROS 패키지의 라이선스를 함께 확인하세요.
