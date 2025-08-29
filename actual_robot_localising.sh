#!/bin/bash
set -e


# 1)launch the robot state publisher for real robot
# 2)run the node from comp_pkg that give timestamp to kinrct_ros2 node and rename them
# 3) launch the depth to laser package to make kinect data to lidar data
# 4) launch kinect ros2 driver 
# 5) localization
tmux new-session -d -s actual_robot_localization "source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot launch_actual_robot.launch.py" \; \
split-window -v "sleep 3; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 run comp_pkg make_timestamp_rgb_for_rtab_odom" \; \
split-window -h "sleep 5; source ~/ros_for_project/articulate_robot/install/setup.bash; ros2 launch my_bot depth_to_lidar.launch.py" \; \
split-window -v "sleep 7; source ~/ros2_kinect_galactic/ws/install/setup.bash; ros2 launch kinect_ros2 pointcloud.launch.py" \; \
split-window -h "sleep 17; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/kinect_fixed/image_raw depth_topic:=/kinect_fixed/depth/image_raw camera_info_topic:=/kinect_fixed/camera_info publish_tf:=true database_path:=~/rtabmap_mymap.db rtabmap_args:='--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true'" \; \
select-layout tiled \; attach

#split-window -h "sleep 10; source ~/ros2_kinect_galactic/ws/install/setup.bash; rm /home/soroush/.ros/rtabmap.db" \; \




# so important: i have to run this in terminal to save the 3d map and move it to another place

# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"

# so important: for saving 2d map:

#ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map

#mv ~/.ros/rtabmap.db ~/rtabmap_mymap.db
