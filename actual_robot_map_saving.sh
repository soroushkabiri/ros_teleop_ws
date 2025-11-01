#!/bin/bash
set -e


# 1)launch the robot state publisher for real robot
# 2)run the node from comp_pkg that give timestamp to kinrct_ros2 node and rename them
# 3) launch the depth to laser package to make kinect data to lidar data
# 4) launch kinect ros2 driver 
# 5) run rtab_map odom to get osometry data from kinect
# 6) get imu data from arduino via serial
# 7) using robot localization package to fuse imu data and visual odometry from rtab map
# 8) make a map with async_mapper_online
tmux new-session -d -s actual_robot_map_save " source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"" \; \
split-window -v "sleep 6; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"" \; \
split-window -h "sleep 10; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map" \; \
split-window -v "sleep 14; source ~/ros2_ws_build_rtabmap/install/setup.bash; ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map" \; \
split-window -h "sleep 18; source ~/ros2_ws_build_rtabmap/install/setup.bash; mv ~/.ros/rtabmap.db ~/rtabmap_mymap.db" \; \
select-layout tiled \; attach





# so important: i have to run this in terminal to save the 3d map and move it to another place

# ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}"

# so important: for saving 2d map:

#ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r /map:=/rtabmap/map

#mv ~/.ros/rtabmap.db ~/rtabmap_mymap.db