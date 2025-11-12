# Actual Differential Drive Robots and Object Transportation

Overview

This repository contains ROS 2 packages designed to connect and control real differential drive robots equipped with IMU sensors, wheel encoders, and ESP32 microcontrollers.
These robots can cooperatively transport a rectangular pallet via passive link mechanisms.
This repository contains hardware codes on esp32 too.

## build
colcon build
source install/setup.bash

## differential drive robots
Each robot uses:
- An ESP32 microcontroller for communication and control
- IMU and encoder feedback for state estimation
- Passive links to attach to and move a rectangular pallet

<img width="341" height="203" alt="diff_drive_real_robot" src="https://github.com/user-attachments/assets/6762cb2e-1928-416d-996b-be1ca8c37b60" />

## rectangular object
The rectangular object being transported is equipped with a Kinect depth camera, used for visual SLAM and visual odometry via the rtabmap package.

<img width="290" height="206" alt="rect_obj_real" src="https://github.com/user-attachments/assets/35f26d86-a5f2-4626-a83f-df752f1400ba" />

## ros2 packages
### imu_pkg

- Collects IMU data from each robot’s ESP32.
- Processes IMU data using the madgwick_filter package.
- Publishes the estimated orientation of each robot.

### esp32_bridge

- Handles communication between each robot’s ESP32 and the central computer.
- Receives desired velocity commands and transmits them to the robot controllers.
- Also collects sensor feedback for monitoring and control.

### mapping
Mapping is performed using RTAB-Map for visual SLAM and localization.
To start mapping:

actual_robot_mapping.sh 

To save the generated map:

actual_robot_map_saving.sh

### transporting the object:
Start the web command node so that the user can define robot goals:

ros2 run esp32_bridge web_command

then run:

actual_robot_localising_and_teleop.sh

<img width="353" height="281" alt="rtab" src="https://github.com/user-attachments/assets/30759fac-e545-4667-8c41-12c1e91baee4" />

### codes on esp folder

The esp folder contains:
- PID motor control code for each wheel
- A simple Kalman filter for initial state estimation and noise reduction

