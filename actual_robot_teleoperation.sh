#!/bin/bash
set -e


# 1) launch esp32_bringup launch to give leader velocity vua joystick and give followers output of consensus
# 2) set pid parameter
# 3) set initial pwm parameter
# 4) launch imu_orientation from mpu_mag_fusion package which it will give orientation of robots
# 5) launch the imu_node that will activated consensus node and publisher of desired velocity for followers

tmux new-session -d -s actual_robot_slam "source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 launch esp32_bridge esp32_bringup.launch.py" \; \
split-window -v "sleep 5; source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"" \; \
split-window -h "sleep 10; source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data:[72, 72]}"" \; \
split-window -v "sleep 12; source ~/Desktop/fusing_mpu_and_magnet_to_madgwich/ws_madgwich/install/setup.bash; ros2 launch mpu_mag_fusion imu_orientation.launch.py" \; \
split-window -h "sleep 15; source ~/esp32_project/5_show_error_and_inverse_kinematic/teleop_esp32/install/setup.bash; ros2 launch esp32_bridge imu_yaw.launch.py" \; \
select-layout tiled \; attach

# ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data:[400, 100, 50]}"
# ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [72, 72]}"
# ros2 param set /initialize_des_publisher kp_angular 0.7
# ros2 param set /initialize_des_publisher ki_angular 0.05
# ros2 param set /initialize_des_publisher max_integral_angular 5.0
