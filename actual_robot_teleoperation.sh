#!/bin/bash
set -e



tmux new-session -d -s actual_robppot_teleop "source install/setup.bash" \; \
split-window -h "sleep 1; source install/setup.bash; ros2 launch esp32_bridge esp32_bringup.launch.py" \; \
split-window -v "sleep 5; source install/setup.bash; ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"" \; \
split-window -h "sleep 10; source install/setup.bash; ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data:[72, 72]}"" \; \
split-window -v "sleep 11; source install/setup.bash; ros2 launch imu_pkg imu_orientation.launch.py" \; \
split-window -h "sleep 14; source install/setup.bash; ros2 launch esp32_bridge imu_yaw.launch.py" \; \
select-layout tiled \; attach

#split-window -v "sleep 5; source install/setup.bash; ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"" \; \
#split-window -h "sleep 10; source install/setup.bash; ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data:[72, 72]}"" \; \

# ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"
# ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [72, 72]}"
# ros2 param set /initialize_des_publisher kp_angular 0.7
# ros2 param set /initialize_des_publisher ki_angular 0.05
# ros2 param set /initialize_des_publisher max_integral_angular 5.0
