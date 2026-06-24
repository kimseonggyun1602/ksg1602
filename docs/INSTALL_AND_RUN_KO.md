# Ubuntu 24.04 초기 상태부터 SLAM 실행까지

이 문서는 ROS 2와 Gazebo가 전혀 설치되지 않은 **Ubuntu 24.04**를 기준으로
다음 파이프라인을 순서대로 실행합니다.

```text
Gazebo 센서 및 구동 데이터
  Wheel odom: /mecanum_drive_controller/odom
  IMU:        /imu/data
  LiDAR:      /scan
  Static TF:  /tf_static
           |
           +--> RTAB-Map ICP odometry --> /icp/odom_raw --> /icp/odom
           |
           +--> robot_localization EKF
                  --> /odometry/filtered
                  --> TF: odom -> base_footprint
                             |
/scan + /tf_static -----------+--> slam_toolbox
                                    --> /map
                                    --> /pose
                                    --> TF: map -> odom
```

각 단계의 확인 명령이 정상 동작한 뒤 다음 단계로 넘어가세요.

## 0. 지원 환경

- Ubuntu 24.04 (Noble), x86-64
- ROS 2 Jazzy
- Gazebo Harmonic (`ros_gz`)
- NVIDIA GPU는 필수가 아닙니다.
- Windows WSL2에서는 Windows 11의 WSLg가 활성화되어야 Gazebo와 RViz 창이
  표시됩니다.

Ubuntu 버전을 확인합니다.

```bash
lsb_release -a
```

`Release: 24.04`가 아니면 이 문서의 패키지 이름을 그대로 사용하지 마세요.

## 1. ROS 2 Jazzy와 필수 패키지 설치

### 방법 A: 설치 스크립트 사용

스크립트를 내려받아 내용을 확인한 뒤 실행합니다.

```bash
curl -fsSL \
  https://raw.githubusercontent.com/kimseonggyun1602/ksg1602/main/scripts/install_ubuntu_24_04_ros2_jazzy.sh \
  -o /tmp/install_ksg_ros2.sh

less /tmp/install_ksg_ros2.sh
bash /tmp/install_ksg_ros2.sh
```

### 방법 B: 명령을 직접 실행

Locale과 Ubuntu Universe 저장소를 설정합니다.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

ROS 공식 APT source 패키지를 설치합니다.

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F\" '{print $4}')

curl -fL \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" \
  -o /tmp/ros2-apt-source.deb

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade -y
```

ROS 2와 이 프로젝트의 의존성을 설치합니다.

```bash
sudo apt install -y \
  git \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
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

`rosdep`을 초기화합니다. 이미 초기화되어 있으면 첫 명령의 오류는 무시해도
됩니다.

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

설치를 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
gz sim --versions
ros2 pkg prefix robot_localization
ros2 pkg prefix slam_toolbox
ros2 pkg prefix rtabmap_odom
```

## 2. 프로젝트 다운로드

```bash
mkdir -p ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git \
  ~/ksg_ws/src/yahboom_rosmaster
```

이미 clone했다면 새로 clone하지 말고 업데이트합니다.

```bash
cd ~/ksg_ws/src/yahboom_rosmaster
git pull --ff-only
```

## 3. 의존성 설치와 빌드

자동 빌드 스크립트를 사용합니다.

```bash
bash ~/ksg_ws/src/yahboom_rosmaster/scripts/build_ksg_ws.sh
```

직접 빌드하려면 다음 명령을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ksg_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --allow-overriding mecanum_drive_controller

source ~/ksg_ws/install/setup.bash
```

패키지가 검색되는지 확인합니다.

```bash
ros2 pkg prefix yahboom_rosmaster_gazebo
ros2 pkg prefix waypoint_follower
```

## 4. 터미널 공통 준비

이후 **새 터미널을 열 때마다** 다음 두 줄을 먼저 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
```

선택적으로 `~/.bashrc` 마지막에 추가할 수 있습니다.

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo '[ -f ~/ksg_ws/install/setup.bash ] && source ~/ksg_ws/install/setup.bash' >> ~/.bashrc
```

## 5. 기존 ROS/Gazebo 프로세스 종료

이전 실행이 남아 있으면 `/clock` 중복, TF 충돌, `TF_OLD_DATA`가 발생합니다.

```bash
pkill -f 'gz sim' || true
pkill -f rviz2 || true
pkill -f ekf_node || true
pkill -f icp_odometry || true
pkill -f odom_covariance_scaler || true
pkill -f async_slam_toolbox_node || true
pkill -f keyboard_teleop || true
sleep 2
```

아무것도 실행하지 않은 상태에서는 `/clock` publisher가 없어야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic info /clock
```

`Unknown topic '/clock'`은 이 단계에서 정상입니다.

## 6. 터미널 1: Gazebo 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 launch yahboom_rosmaster_gazebo \
  yahboom_rosmaster.gazebo.launch.py \
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

`enable_odom_tf:=false`가 중요합니다. Wheel controller의 TF를 끄고 최종
`odom -> base_footprint` TF를 EKF만 발행하게 합니다.

Gazebo 로딩에는 수십 초가 걸릴 수 있습니다.

## 7. 터미널 2: Gazebo 출력 검증

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 topic hz /clock
ros2 control list_controllers
ros2 topic hz /scan
```

Controller 두 개가 `active`여야 합니다.

```text
joint_state_broadcaster       active
mecanum_drive_controller     active
```

`unconfigured`라면 다음 명령을 한 번 실행합니다.

```bash
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state mecanum_drive_controller active
```

센서 메시지를 각각 확인합니다.

```bash
ros2 topic echo /mecanum_drive_controller/odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 run tf2_ros tf2_echo base_footprint imu_link
```

## 8. 터미널 3: ICP LiDAR odometry 실행

ICP는 `/scan`을 연속 정합해 LiDAR odometry를 생성합니다. TF는 EKF가 최종
발행하므로 ICP의 `publish_tf`는 끕니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run rtabmap_odom icp_odometry \
  --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_footprint \
  -p odom_frame_id:=odom \
  -p publish_tf:=false \
  -p wait_for_transform:=0.3 \
  -p approx_sync:=false \
  -r odom:=/icp/odom_raw \
  -r scan_cloud:=/unused_scan_cloud
```

확인:

```bash
ros2 topic hz /icp/odom_raw
ros2 topic echo /icp/odom_raw --once
```

로봇이 정지해 있어도 메시지는 나올 수 있습니다. 실제 이동 추정은 로봇을
움직인 후 확인합니다.

## 9. 터미널 4: ICP covariance 보정

EKF가 ICP 측정의 신뢰도를 판단할 수 있도록 최소 covariance를 부여합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower odom_covariance_scaler \
  --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/icp/odom_raw \
  -p output_topic:=/icp/odom \
  -p pose_xy_variance_floor:=0.02 \
  -p pose_yaw_variance_floor:=0.02 \
  -p twist_xy_variance_floor:=0.04 \
  -p twist_yaw_variance_floor:=0.03
```

확인:

```bash
ros2 topic echo /icp/odom --once
```

## 10. 터미널 5: Wheel + IMU + ICP EKF 실행

사용 설정:

```text
waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

입력은 다음과 같습니다.

```text
Wheel odom: /mecanum_drive_controller/odom
IMU:        /imu/data
ICP odom:   /icp/odom
```

실행:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

출력을 확인합니다.

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

정상 데이터 흐름:

```text
robot_localization
  -> /odometry/filtered
  -> TF: odom -> base_footprint
```

다음 명령으로 `/tf` publisher를 확인할 수 있습니다.

```bash
ros2 topic info /tf -v
```

`odom -> base_footprint`의 동적 TF publisher는 하나여야 합니다.

## 11. 터미널 6: slam_toolbox 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true
```

slam_toolbox의 실제 입력은 다음과 같습니다.

```text
/scan
TF: odom -> base_footprint       (robot_localization 출력)
TF: base_footprint -> laser_frame (/tf_static)
```

출력을 확인합니다.

```bash
ros2 topic info /map -v
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic list | grep -E '^/map$|^/pose$|slam_toolbox'
```

Jazzy 패키지 구성에 따라 `/pose`가 제공되면 다음처럼 확인합니다.

```bash
ros2 topic echo /pose --once
```

`/pose`가 없다면 최종 pose는 아래 TF가 기준입니다.

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

즉 최종 출력은 다음과 같습니다.

```text
Map:  /map
Pose: TF map -> base_footprint
      (= map -> odom x odom -> base_footprint)
```

## 12. 터미널 7: RViz 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
rviz2
```

RViz 설정:

```text
Global Options -> Fixed Frame: map
Add -> Map      -> Topic: /map
Add -> LaserScan -> Topic: /scan
Add -> TF
Add -> RobotModel
```

## 13. 터미널 8: 키보드 조종

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

터미널에 포커스를 둔 상태에서 방향키로 주행합니다. 급회전과 고속 주행은 ICP와
scan matching을 불안정하게 만들 수 있으므로 처음에는 천천히 움직이세요.

## 14. 지도 저장

주행이 끝난 뒤 새 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ksg_results/live_slam

ros2 run nav2_map_server map_saver_cli \
  -f ~/ksg_results/live_slam/map
```

생성 파일:

```text
~/ksg_results/live_slam/map.pgm
~/ksg_results/live_slam/map.yaml
```

## 15. Wheel + IMU 기준 실험으로 변경

ICP 효과를 비교하려면 터미널 3과 4를 종료하고, 기존 EKF도 종료한 다음 아래
설정으로 EKF만 다시 실행합니다.

```bash
pkill -f icp_odometry || true
pkill -f odom_covariance_scaler || true
pkill -f ekf_node || true

source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_basic.yaml
```

두 EKF를 동시에 실행하면 TF가 충돌합니다.

## 16. 전체 입출력 요약

| 노드/패키지 | 입력 | 출력 |
|---|---|---|
| Gazebo + controller | `/mecanum_drive_controller/cmd_vel` | `/mecanum_drive_controller/odom`, `/imu/data`, `/scan`, `/tf_static` |
| `rtabmap_odom/icp_odometry` | `/scan`, static TF | `/icp/odom_raw` |
| covariance scaler | `/icp/odom_raw` | `/icp/odom` |
| `robot_localization/ekf_node` | Wheel odom, IMU, ICP odom | `/odometry/filtered`, `odom -> base_footprint` |
| `slam_toolbox` | `/scan`, `odom -> base_footprint`, static TF | `/map`, `map -> odom`, 최종 pose |

## 17. 자주 발생하는 문제

### Gazebo 또는 RViz 창이 표시되지 않음

일반 Ubuntu에서는 로그인한 데스크톱 세션에서 실행합니다. WSL2에서는 다음을
확인합니다.

```bash
echo $DISPLAY
echo $WAYLAND_DISPLAY
```

둘 다 비어 있다면 WSLg/GUI 환경 문제입니다.

### `/clock`이 없거나 멈춤

```bash
ros2 topic info /clock -v
ros2 topic hz /clock
```

publisher는 하나여야 합니다. Gazebo bridge 설정은 다음 파일에 있습니다.

```text
yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml
```

### Controller가 `unconfigured`

```bash
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state mecanum_drive_controller active
```

### `/scan`이 나오지 않음

```bash
gz topic -l | grep scan
ros2 topic info /scan -v
ros2 topic hz /scan
```

Gazebo가 pause 상태라면 좌측 아래 재생 버튼을 누릅니다.

### `TF_OLD_DATA` 또는 시간이 뒤로 이동했다는 경고

동시에 여러 Gazebo/rosbag이 `/clock`을 publish하거나 이전 프로세스가 남은
상태입니다. 모든 프로세스를 종료하고 5단계부터 다시 시작합니다.

### `odom -> base_footprint` TF가 충돌함

Gazebo launch의 `enable_odom_tf:=false`를 확인하고 EKF가 하나만 실행 중인지
확인합니다.

```bash
ros2 topic info /tf -v
pgrep -af ekf_node
```

### RViz에서 `No map received`

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 lifecycle get /slam_toolbox
```

`slam_toolbox`가 `active`가 아니면 다음을 실행합니다.

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### ICP를 추가했는데 결과가 악화됨

ICP는 항상 개선을 보장하지 않습니다. 다음을 확인하세요.

- Wheel odom이 이미 이상적으로 정확한지
- `/scan`과 TF timestamp가 일치하는지
- ICP covariance를 지나치게 작게 설정하지 않았는지
- 단조로운 벽이나 개방 공간에서 ICP가 잘못 정합되지 않는지
- EKF rejection threshold와 센서별 covariance가 적절한지

먼저 Wheel+IMU 기준 실험을 성공시킨 다음 ICP를 추가하세요.

## 18. 선택 사항: 알고리즘 소스코드 수정

위 절차는 `/opt/ros/jazzy`에 설치된 바이너리 패키지를 사용합니다. YAML
파라미터만 튜닝할 때는 이것으로 충분합니다. EKF 수식, scan matching, loop
closure 또는 graph optimization 구현을 직접 수정하려면 공식 소스를 overlay
workspace에 추가합니다.

```bash
cd ~/ksg_ws/src

git clone -b jazzy-devel \
  https://github.com/cra-ros-pkg/robot_localization.git

git clone -b jazzy \
  https://github.com/SteveMacenski/slam_toolbox.git
```

소스 패키지를 포함해 다시 빌드합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ksg_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --allow-overriding \
  mecanum_drive_controller \
  robot_localization \
  slam_toolbox

source ~/ksg_ws/install/setup.bash
```

overlay 소스가 선택되는지 확인합니다.

```bash
ros2 pkg prefix robot_localization
ros2 pkg prefix slam_toolbox
```

주요 알고리즘 위치:

```text
robot_localization/src/ekf.cpp
  EKF predict/correct

robot_localization/src/filter_base.cpp
  상태 벡터, covariance, process noise 공통 처리

slam_toolbox/lib/karto_sdk/src/Mapper.cpp
  scan matching, loop closure, pose graph

slam_toolbox/solvers/
  pose graph optimization solver
```

권장 순서는 바이너리 패키지로 baseline 재현, source build baseline 확인, YAML
튜닝, C++ 알고리즘 수정, 동일 데이터로 정량 비교입니다.

## 19. 공식 참고 문서

- [ROS 2 Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [robot_localization](https://docs.ros.org/en/ros2_packages/jazzy/api/robot_localization/)
- [slam_toolbox](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)
- [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros)
