#!/bin/bash
set -e


# 1)launch the robot state publisher for real robot
# 2)run the node from comp_pkg that give timestamp to kinrct_ros2 node and rename them
# 3) launch the depth to laser package to make kinect data to lidar data
# 4) launch kinect ros2 driver 
# 5) run rtab_map odom to get osometry data from kinect
# 6) make a map with async_mapper_online
tmux new-session -d -s actual_robot_slam "source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot launch_actual_robot.launch.py" \; \
split-window -v "sleep 3; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 run comp_pkg make_timestamp_rgb_for_rtab_odom" \; \
split-window -h "sleep 5; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot depth_to_lidar.launch.py" \; \
split-window -v "sleep 7; source ~/ros2_kinect_galactic/ws/install/setup.bash; ros2 launch kinect_ros2 pointcloud.launch.py" \; \
split-window -h "sleep 10; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 run rtabmap_odom rgbd_odometry --ros-args  -r /rgb/image:=/kinect_fixed/image_raw -r /depth/image:=/kinect_fixed/depth/image_raw -r /rgb/camera_info:=/kinect_fixed/camera_info" \; \
split-window -v "sleep 15; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot actual_online_async_launch.py" \; \
select-layout tiled \; attach

