# 메카넘 휠 구동 Slip 합성 및 센서 융합 SLAM 비교

이 실험은 실제 wheel-ground 접촉을 재현하는 대신, 바퀴가 헛돌 때 encoder가
실제 body motion보다 큰 이동량을 보고하는 센서 관계를 합성합니다.

```text
고정된 Gazebo GT, IMU, LiDAR scan
                    +
실제보다 이동량을 크게 보고하는 synthetic wheel odom
```

모든 Case는 동일한 raw bag과 `random_seed: 42`를 사용하므로 같은 trajectory,
scan, IMU, GT, slip 오차로 비교됩니다.

## Slip profile

설정 파일:

```text
waypoint_follower/config/wheel_odom_drive_slip.yaml
```

평상시 encoder scale:

```text
x:   1.03
y:   1.20
yaw: 1.01
```

Slip event 구간의 최종 scale:

```text
x:   1.03 × 1.12 = 1.154
y:   1.20 × 1.45 = 1.740
yaw: 1.01 × 1.10 = 1.111
```

따라서 event 구간에서는 횡방향 wheel odom이 실제 이동 증분의 약 1.74배를
보고합니다. 이 값은 특정 실물 로봇의 측정값이 아니라 반복 가능한 비교를
위한 합성 stress profile입니다.

## 공통 준비

```bash
source /opt/ros/jazzy/setup.bash
source ~/ksg_ws/install/setup.bash
```

기본 bag 경로:

```text
~/ajm_bags/factory_lateral_stress_raw
```

다른 위치의 bag을 사용할 때는 각 launch에 다음 인자를 추가합니다.

```bash
bag_path:=/절대/경로/factory_lateral_stress_raw
```

## Case A: Slip Wheel Only

```bash
ros2 launch waypoint_follower replay_slip_wheel_only.launch.py
```

데이터 흐름:

```text
/wheel_odom/slip -> odom→base TF -> slam_toolbox
```

## Case B: Slip Wheel + IMU

```bash
ros2 launch waypoint_follower replay_slip_wheel_imu.launch.py
```

데이터 흐름:

```text
/wheel_odom/slip + /imu/data
-> robot_localization EKF
-> odom→base TF
-> slam_toolbox
```

IMU는 yaw rate를 융합하여 wheel slip에 의한 heading 오차를 줄입니다.

## Case C: Slip Wheel + IMU + ICP LiDAR Odom

```bash
ros2 launch waypoint_follower replay_slip_wheel_imu_icp.launch.py
```

데이터 흐름:

```text
/scan -> RTAB-Map ICP odometry

/wheel_odom/slip + /imu/data + /icp/odom
-> robot_localization EKF
-> odom→base TF
-> slam_toolbox
```

ICP는 scan 간 실제 상대 이동 `x, y, yaw`를 differential measurement로
제공합니다. 같은 LiDAR scan을 ICP와 slam_toolbox가 사용하므로 ICP covariance는
보수적으로 설정되어 있습니다.

## RViz

```text
Fixed Frame: map
Map topic: /map
GT Path: /evaluation/gt_path (green)
SLAM Path: /evaluation/slam_path (red)
```

## 초기 검증 결과

동일한 `factory_lateral_stress_raw` bag에서 확인한 값입니다.

| 구성 | ATE RMSE | Yaw RMSE | Final drift |
|---|---:|---:|---:|
| Slip Wheel only | 0.100 m | 1.074 deg | 0.107 m |
| Slip Wheel + IMU | 0.062 m | 0.540 deg | 0.036 m |
| Slip Wheel + IMU + ICP | 0.043 m | 0.484 deg | 0.021 m |

단일 실행의 초기 결과이므로 최종 발표에서는 여러 seed와 slip 강도로 반복한
평균 및 표준편차를 함께 제시하는 것이 좋습니다.

## 저장된 Map과 Trajectory를 RViz에서 비교

각 결과 폴더에는 다음 파일이 있어야 합니다.

```text
map.pgm
map.yaml
trajectory.csv
localization_metrics.json
```

공통 viewer는 다음 요소를 같은 world 좌표계에 표시합니다.

```text
회색: Gazebo reference map
파란색: slam_toolbox mapping 결과
초록색: Gazebo ground-truth trajectory
빨간색: localization + SLAM 최종 trajectory
```

### Slip Wheel only 결과

```bash
ros2 launch waypoint_follower view_slip_result.launch.py \
  results_dir:=$HOME/ksg_results/slip_wheel_only
```

### Slip Wheel + IMU 결과

```bash
ros2 launch waypoint_follower view_slip_result.launch.py \
  results_dir:=$HOME/ksg_results/slip_wheel_imu
```

### Slip Wheel + IMU + ICP 결과

```bash
ros2 launch waypoint_follower view_slip_result.launch.py \
  results_dir:=$HOME/ksg_results/slip_wheel_imu_icp
```

SLAM map과 trajectory는 localization metrics의 SE(2) alignment를 사용하여
Gazebo world 좌표계에 정렬됩니다. 따라서 단순 원점 차이가 아니라 경로 오차와
지도 벽 형상의 차이를 직접 비교할 수 있습니다.
