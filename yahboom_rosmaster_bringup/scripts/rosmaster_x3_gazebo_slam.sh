#!/bin/bash
# Launch Gazebo simulation + SLAM Toolbox (gz_pose_tf disabled to avoid TF conflict)

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator|map_server|path_publisher|slam_toolbox"
}

trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation (gz_pose_tf disabled for SLAM)..."
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster.gazebo.launch.py \
    enable_odom_tf:=true \
    headless:=False \
    load_controllers:=true \
    world_file:=factory_map_10m.world \
    gz_world_name:=factory_world \
    use_gz_pose_tf:=false \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=-4.45 \
    y:=4.45 \
    z:=0.10 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

echo "Waiting 25 seconds for simulation to initialize..."
sleep 25

echo "Adjusting camera position..."
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0, y: 0, z: 7} orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}}"

echo "Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &

wait
