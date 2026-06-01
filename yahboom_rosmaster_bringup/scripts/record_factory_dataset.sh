#!/bin/bash
# Record one deterministic Gazebo run for repeatable localization and SLAM tests.

set -euo pipefail

OUTPUT_ROOT="${1:-$HOME/ajm_bags}"
RUN_NAME="${2:-factory_raw_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="$OUTPUT_ROOT/$RUN_NAME"
DRIVE_PROFILE="${3:-standard}"
SIM_PID=""
RECORDER_PID=""
FOLLOWER_PID=""

stop_recorder() {
    if [ -z "$RECORDER_PID" ]; then
        return
    fi

    kill -INT "$RECORDER_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do
        if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
            break
        fi
        sleep 1
    done

    if kill -0 "$RECORDER_PID" 2>/dev/null; then
        echo "Recorder did not stop within 10 seconds; terminating it."
        kill -TERM "$RECORDER_PID" 2>/dev/null || true
    fi
    wait "$RECORDER_PID" 2>/dev/null || true
    RECORDER_PID=""

    if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
        echo "Reconstructing missing rosbag metadata..."
        ros2 bag reindex --storage mcap "$BAG_PATH"
    fi
}

cleanup() {
    echo "Stopping dataset recording..."
    if [ -n "$FOLLOWER_PID" ]; then
        kill -INT "$FOLLOWER_PID" 2>/dev/null || true
    fi
    stop_recorder
    if [ -n "$SIM_PID" ]; then
        kill -INT -- "-$SIM_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

mkdir -p "$OUTPUT_ROOT"

if ros2 topic info /clock 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
    echo "An existing simulator is already publishing /clock." >&2
    echo "Stop old Gazebo sessions before recording:" >&2
    echo "  pkill -9 -f '[g]z sim' || true" >&2
    echo "  pkill -9 -f '[r]os2 launch yahboom_rosmaster_gazebo' || true" >&2
    echo "  pkill -9 -f '[r]viz2' || true" >&2
    exit 1
fi

echo "Starting Gazebo factory world..."
setsid ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py \
    enable_odom_tf:=true \
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
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &
SIM_PID=$!

echo "Waiting for simulation topics..."
until ros2 topic list 2>/dev/null | grep -qx '/scan' &&
      ros2 topic list 2>/dev/null | grep -qx '/imu/data' &&
      ros2 topic list 2>/dev/null | grep -qx '/mecanum_drive_controller/odom'; do
    sleep 1
done

echo "Recording raw comparison dataset: $BAG_PATH"
ros2 bag record \
    --output "$BAG_PATH" \
    --storage mcap \
    --use-sim-time \
    --storage-preset-profile zstd_fast \
    --topics \
    /clock \
    /scan \
    /imu/data \
    /mecanum_drive_controller/odom \
    /joint_states \
    /tf_static \
    /gz_world_poses \
    /mecanum_drive_controller/cmd_vel &
RECORDER_PID=$!

sleep 2

if [ "$DRIVE_PROFILE" = "lateral_stress" ]; then
    echo "Starting deterministic lateral-stress waypoint drive..."
    ros2 launch waypoint_follower waypoint_follower.launch.py \
        use_sim_time:=true \
        exit_after_completion:=true \
        fixed_heading_enabled:=true \
        fixed_heading_yaw:=0.0 &
else
    echo "Starting deterministic standard waypoint drive..."
    ros2 launch waypoint_follower waypoint_follower.launch.py \
        use_sim_time:=true \
        exit_after_completion:=true &
fi
FOLLOWER_PID=$!

echo "Waiting until all waypoints are reached..."
wait "$FOLLOWER_PID"
FOLLOWER_PID=""

echo "Route complete. Flushing final sensor messages..."
sleep 2
stop_recorder
echo "Dataset saved: $BAG_PATH"
BAG_INFO="$(ros2 bag info "$BAG_PATH")"
echo "$BAG_INFO"

for topic in /scan /imu/data /mecanum_drive_controller/odom /gz_world_poses; do
    if ! grep -q "Topic: $topic " <<< "$BAG_INFO"; then
        echo "Recorded bag is missing required topic: $topic" >&2
        exit 1
    fi
done

echo "Dataset validation passed."
