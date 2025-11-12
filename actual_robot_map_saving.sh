#!/bin/bash
set -e

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