#!/bin/bash
set -e


# 1)launch the robot state publisher for real robot
# 2)run the node from comp_pkg that give timestamp to kinrct_ros2 node and rename them
# 3) launch the depth to laser package to make kinect data to lidar data
# 4) launch kinect ros2 driver 
# 5) mapping with rtabmap



# Define directory variables
dir_articulate=~/ros_for_project/articulate_robot/install/setup.bash
dir_kinect=~/ros2_kinect_galactic/ws/install/setup.bash
dir_rtabmap=~/ros2_ws_build_rtabmap/install/setup.bash


tmux new-session -d -s actual_robot_slam "source $dir_articulate; ros2 launch my_bot launch_actual_robot.launch.py" \; \
split-window -v "sleep 3; source $dir_articulate; ros2 run comp_pkg make_timestamp_rgb_for_rtab_odom" \; \
split-window -h "sleep 5; source $dir_articulate; ros2 launch my_bot depth_to_lidar.launch.py" \; \
split-window -v "sleep 7; source $dir_kinect; ros2 launch kinect_ros2 pointcloud.launch.py" \; \
split-window -h "sleep 10; source $dir_kinect; rm /home/soroush/.ros/rtabmap.db" \; \
split-window -h "sleep 17; source $dir_rtabmap; ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/kinect_fixed/image_raw depth_topic:=/kinect_fixed/depth/image_raw camera_info_topic:=/kinect_fixed/camera_info  publish_tf:=true" \; \
select-layout tiled \; attach


# so important: we have to run this in terminal to save the 3d map and move it to another place

# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"

# so important: for saving 2d map:

#ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map

#mv ~/.ros/rtabmap.db ~/rtabmap_mymap.db
