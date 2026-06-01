# ROS 2 Jazzy 메카넘 휠 SLAM 재현 가이드

이 저장소는 Ubuntu 24.04와 ROS 2 Jazzy 환경에서 ROSMASTER X3 메카넘 휠
로봇의 Gazebo 시뮬레이션, 위치 추정, SLAM, 정량 평가를 재현하기 위한
소스코드와 설정 파일을 제공합니다.

안정적인 기본 파이프라인은 다음과 같습니다.

```text
Gazebo wheel odom + IMU
  -> robot_localization EKF
  -> TF: odom -> base_footprint

/scan + /tf_static + EKF TF
  -> slam_toolbox
  -> /map
  -> TF: map -> odom
```

ICP LiDAR odometry를 추가하는 선택 실험도 지원합니다.

```text
/scan
  -> RTAB-Map ICP odometry
  -> /icp/odom_raw
  -> covariance scaler
  -> /icp/odom
  -> robot_localization EKF 입력
```

처음 실행할 때는 wheel odom + IMU 기본 구성을 먼저 사용하세요. ICP는 wheel
slip 또는 odometry 오차가 있을 때 비교 실험용으로 추가하는 것이 좋습니다.
Gazebo wheel odom이 이미 이상적으로 정확한 경우에는 ICP가 오히려 결과를
흔들 수 있습니다.

## 1. 요구 환경

- Ubuntu 24.04
- ROS 2 Jazzy
- `ros_gz` 기반 Gazebo Harmonic

필수 패키지를 설치합니다.

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

`rosdep`을 처음 한 번만 초기화합니다.

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 2. 저장소 다운로드 및 빌드

터미널에서 저장소를 내려받습니다.

```bash
mkdir -p ~/ksg_ws/src
cd ~/ksg_ws/src
git clone https://github.com/kimseonggyun1602/ksg1602.git yahboom_rosmaster
```

원본 upstream 저장소만 clone하면 안 됩니다. 이 저장소에는 ROS 2 Jazzy용
Gazebo 설정, EKF 설정, SLAM 실행 파일, 평가 코드가 추가되어 있습니다.

워크스페이스를 빌드합니다.

```bash
cd ~/ksg_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --allow-overriding mecanum_drive_controller
source install/setup.bash
```

## 3. 주요 파일

### Gazebo 시뮬레이션

```text
yahboom_rosmaster_gazebo/launch/yahboom_rosmaster.gazebo.launch.py
yahboom_rosmaster_gazebo/worlds/factory_map_10m.world
yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml
```

Gazebo simulation clock은 다음 설정으로 ROS topic `/clock`에 연결됩니다.

```yaml
- ros_topic_name: "/clock"
  gz_topic_name: "/world/factory_world/clock"
```

### EKF 위치 추정

```text
waypoint_follower/config/ekf_wheel_imu_basic.yaml
waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

### ICP odometry 보조 노드

```text
waypoint_follower/waypoint_follower/odom_covariance_scaler_node.py
```

### 조종 및 평가 도구

```text
waypoint_follower/waypoint_follower/keyboard_teleop_node.py
waypoint_follower/waypoint_follower/trajectory_evaluator_node.py
waypoint_follower/waypoint_follower/map_evaluator.py
```

## 4. 기존 프로세스 종료

새 실험을 시작하기 전에 기존 프로세스를 종료합니다.

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

## 5. 터미널 1: Gazebo 실행

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

controller 로딩에는 약 30초가 걸릴 수 있습니다. 다른 터미널에서 상태를
확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 topic hz /clock
ros2 control list_controllers
ros2 topic hz /scan
```

정상 상태에서는 다음 controller가 `active`로 표시됩니다.

```text
joint_state_broadcaster      active
mecanum_drive_controller     active
```

controller가 계속 `unconfigured`라면 수동으로 활성화합니다.

```bash
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state mecanum_drive_controller active
```

## 6A. 기본 실험: Wheel Odom + IMU EKF

처음에는 이 구성을 권장합니다. ICP 실험을 수행할 때는 이 섹션을 건너뛰고
6B만 실행하세요.

새 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_basic.yaml
```

EKF 출력과 TF를 확인합니다.

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 6B. 선택 실험: Wheel Odom + IMU + ICP EKF

6A의 EKF와 동시에 실행하면 안 됩니다. ICP 효과를 비교할 때만 사용하세요.

### 터미널 2: ICP LiDAR odometry 실행

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

### 터미널 3: ICP covariance 적용

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower odom_covariance_scaler \
  --ros-args \
  -p use_sim_time:=true \
  -p input_topic:=/icp/odom_raw \
  -p output_topic:=/icp/odom
```

ICP 출력이 나오는지 확인합니다.

```bash
ros2 topic echo /icp/odom --once
```

### 터미널 4: ICP를 포함한 EKF 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file \
  ~/ksg_ws/src/yahboom_rosmaster/waypoint_follower/config/ekf_wheel_imu_icp_basic.yaml
```

EKF 출력과 TF를 확인합니다.

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 7. slam_toolbox 실행

6A 또는 6B 중 하나를 실행한 뒤 새 터미널에서 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true
```

SLAM 출력을 확인합니다.

```bash
ros2 topic info /map
ros2 run tf2_ros tf2_echo map base_footprint
```

데이터 흐름은 다음과 같습니다.

```text
/scan + TF: odom -> base_footprint + /tf_static
  -> slam_toolbox
  -> /map
  -> TF: map -> odom
```

## 8. Gazebo GT 경로와 SLAM 경로 생성

실험마다 새로운 결과 폴더를 사용하세요.

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

다음 경로 topic이 publish됩니다.

```text
/evaluation/gt_path
/evaluation/slam_path
```

## 9. RViz 실행 및 설정

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

RViz에서 다음 display를 추가합니다.

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

선택 사항:
  Add -> TF
  Add -> RobotModel
```

화면에 표시되는 결과의 의미는 다음과 같습니다.

```text
회색 occupancy grid: slam_toolbox가 생성한 지도
초록색 경로: /gz_world_poses에서 얻은 Gazebo 실제 이동 경로
빨간색 경로: SLAM 최종 pose인 TF map -> base_footprint
```

## 10. 키보드로 로봇 조종

새 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash

ros2 run waypoint_follower keyboard_teleop \
  --ros-args \
  -p max_linear_vel:=0.6 \
  -p max_angular_vel:=1.5
```

방향키 또는 다음 키를 사용할 수 있습니다.

```text
i: 전진
,: 후진
j: 좌회전
l: 우회전
```

## 11. SLAM 지도 저장 및 정량 평가

주행이 끝난 뒤 지도를 저장합니다.

```bash
mkdir -p ~/ksg_results/live_slam_compare

ros2 run nav2_map_server map_saver_cli \
  -f ~/ksg_results/live_slam_compare/map
```

Gazebo reference map과 SLAM map을 비교합니다.

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

생성되는 파일은 다음과 같습니다.

```text
~/ksg_results/live_slam_compare/map.pgm
~/ksg_results/live_slam_compare/map.yaml
~/ksg_results/live_slam_compare/evaluation/mapping_metrics.json
~/ksg_results/live_slam_compare/evaluation/mapping_overlay.png
```

주요 평가 지표는 다음과 같습니다.

```text
wall_rmse_m: 벽 위치 오차의 RMSE
wall_precision_15cm: 추정 벽 중 reference 벽과 15 cm 이내로 일치하는 비율
wall_recall_15cm: reference 벽 중 추정 지도에서 15 cm 이내로 검출된 비율
occupied_cell_iou_exact: occupied cell의 정확한 겹침 비율
```

## 12. 주요 topic과 TF 요약

```text
Gazebo 출력:
  /mecanum_drive_controller/odom
  /imu/data
  /scan
  /tf_static
  /gz_world_poses

선택적 ICP 출력:
  /icp/odom_raw
  /icp/odom

EKF 출력:
  /odometry/filtered
  TF: odom -> base_footprint

slam_toolbox 출력:
  /map
  TF: map -> odom

최종 SLAM pose:
  TF: map -> base_footprint
```

## 13. 자주 발생하는 문제

### `/clock` topic이 보이지만 데이터가 나오지 않는 경우

```bash
ros2 topic hz /clock
```

출력이 없다면 `yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml`에서 Gazebo
clock topic이 다음과 같이 설정되어 있는지 확인하세요.

```yaml
gz_topic_name: "/world/factory_world/clock"
```

### `/scan` 데이터가 나오지 않는 경우

Gazebo가 실행 중인지, controller가 `active` 상태인지 확인합니다.

```bash
ros2 control list_controllers
ros2 topic hz /scan
```

### ICP를 추가했는데 결과가 더 불안정한 경우

Gazebo wheel odom이 이미 정확하면 정상적으로 발생할 수 있습니다. 먼저 6A의
wheel odom + IMU baseline을 사용하세요. ICP는 wheel slip 또는 odometry
degradation을 넣은 비교 실험에서 보수적인 covariance와 함께 사용하세요.

## 14. 선택 사항: 알고리즘 내부 소스코드 수정

위의 기본 실행 방법은 Ubuntu에 설치된 ROS 패키지를 사용합니다.

```text
/opt/ros/jazzy
```

YAML 파라미터만 튜닝할 때는 추가 작업이 필요하지 않습니다. EKF 수식,
scan matching, loop closure, graph optimization 등 패키지 내부 알고리즘을
직접 수정하려면 공식 소스코드를 workspace에 추가한 뒤 다시 빌드하세요.

### 공식 소스코드 추가

```bash
cd ~/ksg_ws/src

git clone -b jazzy-devel \
  https://github.com/cra-ros-pkg/robot_localization.git

git clone -b jazzy \
  https://github.com/SteveMacenski/slam_toolbox.git
```

### overlay workspace 다시 빌드

```bash
cd ~/ksg_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --allow-overriding \
  mecanum_drive_controller \
  robot_localization \
  slam_toolbox

source install/setup.bash
```

source build가 우선 적용되는지 확인합니다.

```bash
ros2 pkg prefix robot_localization
ros2 pkg prefix slam_toolbox
```

정상적으로 overlay가 적용되면 `/opt/ros/jazzy` 대신 다음과 유사한 경로가
출력됩니다.

```text
/home/<사용자명>/ksg_ws/install/robot_localization
/home/<사용자명>/ksg_ws/install/slam_toolbox
```

### 주요 수정 위치

```text
robot_localization/src/ekf.cpp
  EKF prediction 및 correction 수식

robot_localization/src/filter_base.cpp
  공통 상태 벡터, covariance, process noise 처리

slam_toolbox/lib/karto_sdk/src/Mapper.cpp
  scan matching, loop closure, pose graph 관련 처리

slam_toolbox/solvers/
  graph optimization solver 구현
```

권장 연구 순서는 다음과 같습니다.

```text
1. apt 설치본으로 baseline 재현
2. 공식 소스코드를 추가하고 source build
3. source build에서도 baseline 결과가 동일한지 확인
4. YAML 파라미터 튜닝
5. 필요한 C++ 내부 알고리즘 수정
6. 기존 baseline과 map 및 trajectory 지표 비교
```
