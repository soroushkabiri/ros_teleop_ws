#!/bin/bash
set -e
# Kill any old tmux sessions
tmux kill-server || true
sleep 3

# 1)launch the robot state publisher for real robot
# 2)run the node from comp_pkg that give timestamp to kinrct_ros2 node and rename them
# 3) launch the depth to laser package to make kinect data to lidar data
# 4) launch kinect ros2 driver 
# 5) localization
# 6) record data
# 7) compute closest obstacle to object in map


##### we need to run webcommand beforehand


tmux new-session -d -s actual_robot_localization "source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot launch_actual_robot.launch.py" \; \
split-window -v "sleep 3; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 run comp_pkg make_timestamp_rgb_for_rtab_odom" \; \
split-window -h "sleep 5; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot depth_to_lidar.launch.py" \; \
split-window -v "sleep 7; source ~/ros2_kinect_galactic/ws/install/setup.bash; ros2 launch kinect_ros2 pointcloud.launch.py" \; \
split-window -h "sleep 17; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/kinect_fixed/image_raw depth_topic:=/kinect_fixed/depth/image_raw camera_info_topic:=/kinect_fixed/camera_info publish_tf:=true database_path:=~/rtabmap_mymap.db rtabmap_args:='--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true'" \; \
split-window -h "sleep 18; source install/setup.bash; ros2 run esp32_bridge bag_recorder" \; \
split-window -v "sleep 22; source install/setup.bash; ros2 run esp32_bridge compute_closest_obstacle" \; \
select-layout tiled 


tmux new-session -d -s actual_robppot_teleop "source install/setup.bash" \; \
split-window -h "sleep 30; source install/setup.bash; ros2 launch esp32_bridge esp32_bringup.launch.py" \; \
split-window -v "sleep 35; source install/setup.bash; ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"" \; \
split-window -h "sleep 40; source install/setup.bash; ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data:[72, 72]}"" \; \
split-window -v "sleep 42; source install/setup.bash; ros2 launch imu_pkg imu_orientation.launch.py" \; \
split-window -h "sleep 45; source install/setup.bash; ros2 launch esp32_bridge imu_yaw.launch.py" \; \
select-layout tiled

#split-window -v "sleep 42; source ~/Desktop/fusing_mpu_and_magnet_to_madgwich/ws_madgwich/install/setup.bash; ros2 launch mpu_mag_fusion imu_orientation.launch.py" \; \

#split-window -h "sleep 50; source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 run esp32_bridge fuzzy_planner" \; \


#split-window -v "sleep 47; source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 run esp32_bridge web_command" \; \
# Open both sessions in separate terminal windows
gnome-terminal -- bash -c "tmux attach -t actual_robot_localization" &
gnome-terminal -- bash -c "tmux attach -t actual_robppot_teleop" &


# ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"
# ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [72, 72]}"
# ros2 param set /initialize_des_publisher kp_angular 0.7
# ros2 param set /initialize_des_publisher ki_angular 0.05
# ros2 param set /initialize_des_publisher max_integral_angular 5.0
#ros2 topic pub /rtabmap/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}"
