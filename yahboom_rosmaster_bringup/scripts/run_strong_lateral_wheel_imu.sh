#!/bin/bash
# Run SLAM with degraded wheel odometry fused with IMU by robot_localization.

set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/ksg_ws/install/setup.bash"
set -u

BAG_PATH="${1:-$HOME/ajm_bags/factory_lateral_stress_raw}"
RESULTS_DIR="${2:-$HOME/ksg_results/strong_lateral_wheel_imu}"
USE_RVIZ="${USE_RVIZ:-false}"
SHOW_RESULT_RVIZ="${SHOW_RESULT_RVIZ:-true}"
REFERENCE_YAML="$HOME/ksg_ws/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/rviz/rviz_maps/my_factory_map.yaml"
LAUNCH_PID=""

cleanup() {
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
    echo "Bag metadata not found: $BAG_PATH" >&2
    exit 1
fi

if ros2 topic info /clock 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
    echo "Another simulator or rosbag is already publishing /clock." >&2
    echo "Stop old launches before starting the strong wheel and IMU run." >&2
    exit 1
fi

mkdir -p "$RESULTS_DIR"

echo "Evaluating the shared raw input bag before wheel odom degradation..."
ros2 run waypoint_follower raw_bag_evaluator \
    --bag "$BAG_PATH" \
    --output-dir "$RESULTS_DIR/raw_bag_analysis"

echo "============================================================"
echo "Strong lateral wheel and IMU SLAM"
echo "Bag:     $BAG_PATH"
echo "Results: $RESULTS_DIR"
echo "Wheel odom degradation: enabled"
echo "Lateral scale:          0.80"
echo "Slip event y scale:     0.80 x 0.45 = 0.36"
echo "Slip event ranges:      4.0-5.5 m, 10.0-11.5 m"
echo "EKF input 0:            /wheel_odom/degraded"
echo "EKF input 1:            /imu/data"
echo "EKF TF output:          odom -> base_footprint"
echo "============================================================"

setsid ros2 launch waypoint_follower replay_wheel_imu_slam.launch.py \
    bag_path:="$BAG_PATH" \
    degradation_enabled:=true \
    results_dir:="$RESULTS_DIR" \
    use_rviz:="$USE_RVIZ" &
LAUNCH_PID=$!

echo "Waiting for nodes and rosbag playback..."
sleep 32

echo "Saving wheel odom degradation diagnostics..."
ros2 topic echo /wheel_odom_degrader/status --once \
    > "$RESULTS_DIR/wheel_odom_degrader_status.yaml" || true

echo "Saving occupancy-grid map..."
ros2 run nav2_map_server map_saver_cli \
    -f "$RESULTS_DIR/map"

echo "Stopping SLAM nodes to flush trajectory metrics..."
kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

if [ ! -f "$RESULTS_DIR/localization_metrics.json" ]; then
    echo "Localization metrics were not written." >&2
    exit 1
fi

if [ ! -f "$RESULTS_DIR/map.yaml" ]; then
    echo "Map was not saved." >&2
    exit 1
fi

echo "Evaluating occupancy-grid map against Gazebo reference map..."
ros2 run waypoint_follower map_evaluator \
    --reference-yaml "$REFERENCE_YAML" \
    --slam-yaml "$RESULTS_DIR/map.yaml" \
    --output-dir "$RESULTS_DIR/evaluation" \
    --alignment-json "$RESULTS_DIR/localization_metrics.json"

echo "Rendering trajectory comparison plot..."
ros2 run waypoint_follower trajectory_plotter \
    --results-dir "$RESULTS_DIR"

echo
echo "============================================================"
echo "Strong lateral wheel and IMU run completed"
echo "Map image:            $RESULTS_DIR/map.pgm"
echo "SLAM map metadata:    $RESULTS_DIR/map.yaml"
echo "Trajectory samples:   $RESULTS_DIR/trajectory.csv"
echo "Localization metrics: $RESULTS_DIR/localization_metrics.json"
echo "Mapping metrics:      $RESULTS_DIR/evaluation/mapping_metrics.json"
echo "Mapping overlay:      $RESULTS_DIR/evaluation/mapping_overlay.png"
echo "Degrader diagnostics: $RESULTS_DIR/wheel_odom_degrader_status.yaml"
echo "Trajectory plot:      $RESULTS_DIR/trajectory_plot.png"
echo "============================================================"

if [ "$SHOW_RESULT_RVIZ" = "true" ]; then
    echo
    echo "Opening the persistent saved-result RViz view."
    echo "Close RViz or press Ctrl+C in this terminal when finished."
    exec ros2 launch waypoint_follower \
        view_strong_lateral_wheel_imu_result.launch.py \
        results_dir:="$RESULTS_DIR"
fi
